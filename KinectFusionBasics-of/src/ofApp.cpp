#include "ofApp.h"

// Safe release for interfaces
template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
    if ( pInterfaceToRelease != NULL )
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix( Matrix4 &mat )
{
    mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
    mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
    mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
    mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

void UpdateIntrinsics( NUI_FUSION_IMAGE_FRAME * pImageFrame, NUI_FUSION_CAMERA_PARAMETERS * params )
{
    if ( pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr )
    {
        pImageFrame->pCameraParameters->focalLengthX = params->focalLengthX;
        pImageFrame->pCameraParameters->focalLengthY = params->focalLengthY;
        pImageFrame->pCameraParameters->principalPointX = params->principalPointX;
        pImageFrame->pCameraParameters->principalPointY = params->principalPointY;
    }

    // Confirm we are called correctly
    _ASSERT( pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr );
}

#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif

#ifndef SAFE_FUSION_RELEASE_IMAGE_FRAME
#define SAFE_FUSION_RELEASE_IMAGE_FRAME(p) { if (p) { static_cast<void>(NuiFusionReleaseImageFrame(p)); (p)=NULL; } }
#endif

ofApp::ofApp():
    //m_pD2DFactory( nullptr ),
    //m_pDrawDepth( nullptr ),
    m_pVolume( nullptr ),
    m_pNuiSensor( nullptr ),
    m_cDepthImagePixels( 0 ),
    m_bMirrorDepthFrame( false ),
    m_bTranslateResetPoseByMinDepthThreshold( true ),
    m_bAutoResetReconstructionWhenLost( false ),
    m_bAutoResetReconstructionOnTimeout( true ),
    m_lastFrameTimeStamp( 0 ),
    m_bResetReconstruction( false ),
    m_cLostFrameCounter( 0 ),
    m_bTrackingFailed( false ),
    m_cFrameCounter( 0 ),
    m_fStartTime( 0 ),
    m_pMapper( nullptr ),
    m_pDepthImagePixelBuffer( nullptr ),
    m_pDepthDistortionMap( nullptr ),
    m_pDepthDistortionLT( nullptr ),
    m_pDepthFloatImage( nullptr ),
    m_pPointCloud( nullptr ),
    m_pShadedSurface( nullptr ),
    m_bInitializeError( false ),
    m_pDepthFrameReader( NULL ),
    m_coordinateMappingChangedEvent( NULL ),
    m_bHaveValidCameraParameters( false )
{
    // Get the depth frame size from the NUI_IMAGE_RESOLUTION enum
    // You can use NUI_IMAGE_RESOLUTION_640x480 or NUI_IMAGE_RESOLUTION_320x240 in this sample
    // Smaller resolutions will be faster in per-frame computations, but show less detail in reconstructions.
    m_cDepthWidth = NUI_DEPTH_RAW_WIDTH;
    m_cDepthHeight = NUI_DEPTH_RAW_HEIGHT;
    m_cDepthImagePixels = m_cDepthWidth * m_cDepthHeight;

    // create heap storage for depth pixel data in RGBX format
    //m_pDepthRGBX = new BYTE[m_cDepthImagePixels * cBytesPerPixel];

    // Define a cubic Kinect Fusion reconstruction volume,
    // with the Kinect at the center of the front face and the volume directly in front of Kinect.
    m_reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel    
    m_reconstructionParams.voxelCountX = 384;   // 384 / 256vpm = 1.5m wide reconstruction
    m_reconstructionParams.voxelCountY = 384;   // Memory = 384*384*384 * 4bytes per voxel
    m_reconstructionParams.voxelCountZ = 384;   // This will require a GPU with at least 256MB

    // These parameters are for optionally clipping the input depth image 
    m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
    m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

    // This parameter is the temporal averaging parameter for depth integration into the reconstruction
    m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes

    // This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
    // too slow for real-time processing.
    m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

    // If GPU processing is selected, we can choose the index of the device we would like to
    // use for processing by setting this zero-based index parameter. Note that setting -1 will cause
    // automatic selection of the most suitable device (specifically the DirectX11 compatible device 
    // with largest memory), which is useful in systems with multiple GPUs when only one reconstruction
    // volume is required. Note that the automatic choice will not load balance across multiple 
    // GPUs, hence users should manually select GPU indices when multiple reconstruction volumes 
    // are required, each on a separate device.
    m_deviceIndex = -1;    // automatically choose device index for processing

    SetIdentityMatrix( m_worldToCameraTransform );
    SetIdentityMatrix( m_defaultWorldToVolumeTransform );

    // We don't know these at object creation time, so we use nominal values.
    // These will later be updated in response to the CoordinateMappingChanged event.
    m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
    m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
    m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
    m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
}


//--------------------------------------------------------------
void ofApp::setup(){

    auto hr = CreateFirstConnected();
    if ( SUCCEEDED( hr ) )
    {
        hr = InitializeKinectFusion();
    }

    colorTex.allocate( m_cDepthWidth, m_cDepthHeight, GL_RGBA );

    if ( FAILED( hr ) ) {
        SetStatusMessage( L"No ready Kinect found!" );
    }
}

HRESULT ofApp::CreateFirstConnected()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor( &m_pNuiSensor );
    if ( FAILED( hr ) )
    {
        return hr;
    }

    if ( m_pNuiSensor )
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pNuiSensor->Open();

        if ( SUCCEEDED( hr ) )
        {
            hr = m_pNuiSensor->get_CoordinateMapper( &m_pMapper );
        }

        if ( SUCCEEDED( hr ) )
        {
            hr = m_pNuiSensor->get_DepthFrameSource( &pDepthFrameSource );
        }

        if ( SUCCEEDED( hr ) )
        {
            hr = m_pMapper->SubscribeCoordinateMappingChanged( &m_coordinateMappingChangedEvent );
        }

        if ( SUCCEEDED( hr ) )
        {
            hr = pDepthFrameSource->OpenReader( &m_pDepthFrameReader );
        }

        SafeRelease( pDepthFrameSource );
    }

    if ( nullptr == m_pNuiSensor || FAILED( hr ) )
    {
        SetStatusMessage( L"No ready Kinect found!" );
        return E_FAIL;
    }

    return hr;
}

HRESULT ofApp::InitializeKinectFusion()
{
    HRESULT hr = S_OK;

    // Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
    WCHAR description[MAX_PATH];
    WCHAR instancePath[MAX_PATH];
    UINT memorySize = 0;

    if ( FAILED( hr = NuiFusionGetDeviceInfo(
        m_processorType,
        m_deviceIndex,
        &description[0],
        ARRAYSIZE( description ),
        &instancePath[0],
        ARRAYSIZE( instancePath ),
        &memorySize ) ) )
    {
        if ( hr == E_NUI_BADINDEX )
        {
            // This error code is returned either when the device index is out of range for the processor 
            // type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
            // for the device index in the parameters, this indicates that there is no DirectX11 capable 
            // device. The options for users in this case are to either install a DirectX11 capable device
            // (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
            // reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
            SetStatusMessage( L"No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction." );
        }
        else
        {
            SetStatusMessage( L"Failed in call to NuiFusionGetDeviceInfo." );
        }
        return hr;
    }

    // Create the Kinect Fusion Reconstruction Volume
    hr = NuiFusionCreateReconstruction(
        &m_reconstructionParams,
        m_processorType, m_deviceIndex,
        &m_worldToCameraTransform,
        &m_pVolume );

    if ( FAILED( hr ) )
    {
        if ( E_NUI_GPU_FAIL == hr )
        {
            WCHAR buf[MAX_PATH];
            swprintf_s( buf, ARRAYSIZE( buf ), L"Device %d not able to run Kinect Fusion, or error initializing.", m_deviceIndex );
            SetStatusMessage( buf );
        }
        else if ( E_NUI_GPU_OUTOFMEMORY == hr )
        {
            WCHAR buf[MAX_PATH];
            swprintf_s( buf, ARRAYSIZE( buf ), L"Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.", m_deviceIndex );
            SetStatusMessage( buf );
        }
        else if ( NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType )
        {
            WCHAR buf[MAX_PATH];
            swprintf_s( buf, ARRAYSIZE( buf ), L"Failed to initialize Kinect Fusion reconstruction volume on device %d.", m_deviceIndex );
            SetStatusMessage( buf );
        }
        else
        {
            SetStatusMessage( L"Failed to initialize Kinect Fusion reconstruction volume on CPU." );
        }

        return hr;
    }

    // Save the default world to volume transformation to be optionally used in ResetReconstruction
    hr = m_pVolume->GetCurrentWorldToVolumeTransform( &m_defaultWorldToVolumeTransform );
    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Failed in call to GetCurrentWorldToVolumeTransform." );
        return hr;
    }

    if ( m_bTranslateResetPoseByMinDepthThreshold )
    {
        // This call will set the world-volume transformation
        hr = ResetReconstruction();
        if ( FAILED( hr ) )
        {
            return hr;
        }
    }

    // Frames generated from the depth input
    hr = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_FLOAT, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pDepthFloatImage );
    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion image." );
        return hr;
    }

    // Create images to raycast the Reconstruction Volume
    hr = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pPointCloud );
    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion image." );
        return hr;
    }

    // Create images to raycast the Reconstruction Volume
    hr = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pShadedSurface );
    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion image." );
        return hr;
    }

    _ASSERT( m_pDepthImagePixelBuffer == nullptr );
    m_pDepthImagePixelBuffer = new(std::nothrow) UINT16[m_cDepthImagePixels];
    if ( nullptr == m_pDepthImagePixelBuffer )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion depth image pixel buffer." );
        return hr;
    }

    _ASSERT( m_pDepthDistortionMap == nullptr );
    m_pDepthDistortionMap = new(std::nothrow) DepthSpacePoint[m_cDepthImagePixels];
    if ( nullptr == m_pDepthDistortionMap )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion depth image distortion buffer." );
        return E_OUTOFMEMORY;
    }

    SAFE_DELETE_ARRAY( m_pDepthDistortionLT );
    m_pDepthDistortionLT = new(std::nothrow) UINT[m_cDepthImagePixels];

    if ( nullptr == m_pDepthDistortionLT )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion depth image distortion Lookup Table." );
        return E_OUTOFMEMORY;
    }

    // If we have valid parameters, let's go ahead and use them.
    if ( m_cameraParameters.focalLengthX != 0 )
    {
        SetupUndistortion();
    }

    m_fStartTime = m_timer.AbsoluteTime();

    // Set an introductory message
    SetStatusMessage( L"Click \'Reset Reconstruction\' to clear!" );

    return hr;
}

HRESULT ofApp::SetupUndistortion()
{
    HRESULT hr = E_UNEXPECTED;

    if ( m_cameraParameters.principalPointX != 0 )
    {

        CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
        {
            /*LT*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
            /*RT*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
            /*LB*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f },
            /*RB*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }
        };

        for ( UINT rowID = 0; rowID < m_cDepthHeight; rowID++ )
        {
            const float rowFactor = float( rowID ) / float( m_cDepthHeight - 1 );
            const CameraSpacePoint rowStart =
            {
                cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
                cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
                1.f
            };

            const CameraSpacePoint rowEnd =
            {
                cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
                cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
                1.f
            };

            const float stepFactor = 1.f / float( m_cDepthWidth - 1 );
            const CameraSpacePoint rowDelta =
            {
                (rowEnd.X - rowStart.X) * stepFactor,
                (rowEnd.Y - rowStart.Y) * stepFactor,
                0
            };

            _ASSERT( m_cDepthWidth == NUI_DEPTH_RAW_WIDTH );
            CameraSpacePoint cameraCoordsRow[NUI_DEPTH_RAW_WIDTH];

            CameraSpacePoint currentPoint = rowStart;
            for ( UINT i = 0; i < m_cDepthWidth; i++ )
            {
                cameraCoordsRow[i] = currentPoint;
                currentPoint.X += rowDelta.X;
                currentPoint.Y += rowDelta.Y;
            }

            hr = m_pMapper->MapCameraPointsToDepthSpace( m_cDepthWidth, cameraCoordsRow, m_cDepthWidth, &m_pDepthDistortionMap[rowID * m_cDepthWidth] );
            if ( FAILED( hr ) )
            {
                SetStatusMessage( L"Failed to initialize Kinect Coordinate Mapper." );
                return hr;
            }
        }

        if ( nullptr == m_pDepthDistortionLT )
        {
            SetStatusMessage( L"Failed to initialize Kinect Fusion depth image distortion Lookup Table." );
            return E_OUTOFMEMORY;
        }

        UINT* pLT = m_pDepthDistortionLT;
        for ( UINT i = 0; i < m_cDepthImagePixels; i++, pLT++ )
        {
            //nearest neighbor depth lookup table 
            UINT x = UINT( m_pDepthDistortionMap[i].X + 0.5f );
            UINT y = UINT( m_pDepthDistortionMap[i].Y + 0.5f );

            *pLT = (x < m_cDepthWidth && y < m_cDepthHeight) ? x + y * m_cDepthWidth : UINT_MAX;
        }
        m_bHaveValidCameraParameters = true;
    }
    else
    {
        m_bHaveValidCameraParameters = false;
    }
    return S_OK;
}

HRESULT ofApp::ResetReconstruction()
{
    if ( nullptr == m_pVolume )
    {
        return E_FAIL;
    }

    HRESULT hr = S_OK;

    SetIdentityMatrix( m_worldToCameraTransform );

    // Translate the reconstruction volume location away from the world origin by an amount equal
    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
    // If set false, the default world origin is set to the center of the front face of the 
    // volume, which has the effect of locating the volume directly in front of the initial camera
    // position with the +Z axis into the volume along the initial camera direction of view.
    if ( m_bTranslateResetPoseByMinDepthThreshold )
    {
        Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

        // Translate the volume in the Z axis by the minDepthThreshold distance
        float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
        worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);

        hr = m_pVolume->ResetReconstruction( &m_worldToCameraTransform, &worldToVolumeTransform );
    }
    else
    {
        hr = m_pVolume->ResetReconstruction( &m_worldToCameraTransform, nullptr );
    }

    m_cLostFrameCounter = 0;
    m_cFrameCounter = 0;
    m_fStartTime = m_timer.AbsoluteTime();

    if ( SUCCEEDED( hr ) )
    {
        m_bTrackingFailed = false;

        SetStatusMessage( L"Reconstruction has been reset." );
    }
    else
    {
        SetStatusMessage( L"Failed to reset reconstruction." );
    }

    return hr;
}

//--------------------------------------------------------------
void ofApp::update(){
    if ( nullptr == m_pNuiSensor )
    {
        return;
    }

    if ( m_coordinateMappingChangedEvent != NULL &&
        WAIT_OBJECT_0 == WaitForSingleObject( (HANDLE)m_coordinateMappingChangedEvent, 0 ) )
    {
        OnCoordinateMappingChanged();
        ResetEvent( (HANDLE)m_coordinateMappingChangedEvent );
    }

    if ( !m_bHaveValidCameraParameters )
    {
        return;
    }

    m_bResetReconstruction = false;

    if ( !m_pDepthFrameReader )
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame( &pDepthFrame );

    if ( SUCCEEDED( hr ) )
    {
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;
        INT64 currentTimestamp = 0;

        hr = pDepthFrame->get_RelativeTime( &currentTimestamp );
        if ( SUCCEEDED( hr ) && currentTimestamp - m_lastFrameTimeStamp > cResetOnTimeStampSkippedMilliseconds * 10000
            && 0 != m_lastFrameTimeStamp )
        {
            m_bResetReconstruction = true;
        }
        m_lastFrameTimeStamp = currentTimestamp;

        if ( SUCCEEDED( hr ) )
        {
            hr = pDepthFrame->AccessUnderlyingBuffer( &nBufferSize, &pBuffer );
        }

        if ( SUCCEEDED( hr ) )
        {
            //copy and remap depth
            const UINT bufferLength = m_cDepthImagePixels;
            UINT16 * pDepth = m_pDepthImagePixelBuffer;
            for ( UINT i = 0; i < bufferLength; i++, pDepth++ )
            {
                const UINT id = m_pDepthDistortionLT[i];
                *pDepth = id < bufferLength ? pBuffer[id] : 0;
            }

            ProcessDepth();
        }
    }

    SafeRelease( pDepthFrame );
}

void ofApp::ProcessDepth()
{
    if ( m_bInitializeError )
    {
        return;
    }

    HRESULT hr = S_OK;

    // To enable playback of a .xef file through Kinect Studio and reset of the reconstruction
    // if the .xef loops, we test for when the frame timestamp has skipped a large number. 
    // Note: this will potentially continually reset live reconstructions on slow machines which
    // cannot process a live frame in less time than the reset threshold. Increase the number of
    // milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
    if ( m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0 && m_bResetReconstruction )
    {
        ResetReconstruction();

        if ( FAILED( hr ) )
        {
            return;
        }
    }

    // Return if the volume is not initialized
    if ( nullptr == m_pVolume )
    {
        SetStatusMessage( L"Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting." );
        return;
    }

    ////////////////////////////////////////////////////////
    // Depth to DepthFloat

    // Convert the pixels describing extended depth as unsigned short type in millimeters to depth
    // as floating point type in meters.
    hr = m_pVolume->DepthToDepthFloatFrame( m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof( UINT16 ), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame );

    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." );
        return;
    }

    ////////////////////////////////////////////////////////
    // ProcessFrame

    // Perform the camera tracking and update the Kinect Fusion Volume
    // This will create memory on the GPU, upload the image, run camera tracking and integrate the
    // data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
    // parameter will use and update the internal camera pose.
    hr = m_pVolume->ProcessFrame( m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, nullptr, &m_worldToCameraTransform );

    // Test to see if camera tracking failed. 
    // If it did fail, no data integration or raycast for reference points and normals will have taken 
    //  place, and the internal camera pose will be unchanged.
    if ( FAILED( hr ) )
    {
        if ( hr == E_NUI_FUSION_TRACKING_ERROR )
        {
            m_cLostFrameCounter++;
            m_bTrackingFailed = true;
            SetStatusMessage( L"Kinect Fusion camera tracking failed! Align the camera to the last tracked position. " );
        }
        else
        {
            SetStatusMessage( L"Kinect Fusion ProcessFrame call failed!" );
            return;
        }
    }
    else
    {
        Matrix4 calculatedCameraPose;
        hr = m_pVolume->GetCurrentWorldToCameraTransform( &calculatedCameraPose );

        if ( SUCCEEDED( hr ) )
        {
            // Set the pose
            m_worldToCameraTransform = calculatedCameraPose;
            m_cLostFrameCounter = 0;
            m_bTrackingFailed = false;
        }
    }

    if ( m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames )
    {
        // Automatically clear volume and reset tracking if tracking fails
        hr = ResetReconstruction();

        if ( FAILED( hr ) )
        {
            return;
        }

        // Set bad tracking message
        SetStatusMessage( L"Kinect Fusion camera tracking failed, automatically reset volume." );
    }

    ////////////////////////////////////////////////////////
    // CalculatePointCloud

    // Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
    hr = m_pVolume->CalculatePointCloud( m_pPointCloud, &m_worldToCameraTransform );

    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Kinect Fusion CalculatePointCloud call failed." );
        return;
    }

    ////////////////////////////////////////////////////////
    // ShadePointCloud and render

    hr = NuiFusionShadePointCloud( m_pPointCloud, &m_worldToCameraTransform, nullptr, m_pShadedSurface, nullptr );

    if ( FAILED( hr ) )
    {
        SetStatusMessage( L"Kinect Fusion NuiFusionShadePointCloud call failed." );
        return;
    }

    // Draw the shaded raycast volume image
    BYTE * pBuffer = m_pShadedSurface->pFrameBuffer->pBits;

    for ( int i = 0; i < m_cDepthWidth * m_cDepthHeight; ++i ){
        int index = i * 4;
        pBuffer[index + 3] = 255;
    }

    // Draw the data with Direct2D
    colorTex.loadData( pBuffer, m_cDepthWidth, m_cDepthHeight, GL_RGBA );
    //m_pDrawDepth->Draw( pBuffer, m_cDepthWidth * m_cDepthHeight * cBytesPerPixel );

    ////////////////////////////////////////////////////////
    // Periodically Display Fps

    // Update frame counter
    m_cFrameCounter++;

    // Display fps count approximately every cTimeDisplayInterval seconds
    double elapsed = m_timer.AbsoluteTime() - m_fStartTime;
    if ( (int)elapsed >= cTimeDisplayInterval )
    {
        double fps = (double)m_cFrameCounter / elapsed;

        // Update status display
        if ( !m_bTrackingFailed )
        {
            WCHAR str[MAX_PATH];
            swprintf_s( str, ARRAYSIZE( str ), L"Fps: %5.2f", fps );
            SetStatusMessage( str );
        }

        m_cFrameCounter = 0;
        m_fStartTime = m_timer.AbsoluteTime();
    }
}

HRESULT ofApp::OnCoordinateMappingChanged()
{
    HRESULT hr = E_UNEXPECTED;

    // Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
    CameraIntrinsics intrinsics = {};

    m_pMapper->GetDepthCameraIntrinsics( &intrinsics );

    float focalLengthX = intrinsics.FocalLengthX / NUI_DEPTH_RAW_WIDTH;
    float focalLengthY = intrinsics.FocalLengthY / NUI_DEPTH_RAW_HEIGHT;
    float principalPointX = intrinsics.PrincipalPointX / NUI_DEPTH_RAW_WIDTH;
    float principalPointY = intrinsics.PrincipalPointY / NUI_DEPTH_RAW_HEIGHT;

    if ( m_cameraParameters.focalLengthX == focalLengthX && m_cameraParameters.focalLengthY == focalLengthY &&
        m_cameraParameters.principalPointX == principalPointX && m_cameraParameters.principalPointY == principalPointY )
        return S_OK;

    m_cameraParameters.focalLengthX = focalLengthX;
    m_cameraParameters.focalLengthY = focalLengthY;
    m_cameraParameters.principalPointX = principalPointX;
    m_cameraParameters.principalPointY = principalPointY;

    _ASSERT( m_cameraParameters.focalLengthX != 0 );

    UpdateIntrinsics( m_pDepthFloatImage, &m_cameraParameters );
    UpdateIntrinsics( m_pPointCloud, &m_cameraParameters );
    UpdateIntrinsics( m_pShadedSurface, &m_cameraParameters );

    if ( nullptr == m_pDepthDistortionMap )
    {
        SetStatusMessage( L"Failed to initialize Kinect Fusion depth image distortion buffer." );
        return E_OUTOFMEMORY;
    }

    hr = SetupUndistortion();
    return hr;
}

//--------------------------------------------------------------
void ofApp::draw(){
    colorTex.draw( 0, 0, m_cDepthWidth, m_cDepthHeight );
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::SetStatusMessage( WCHAR * szMessage )
{
    std::wcout << szMessage << std::endl;
}

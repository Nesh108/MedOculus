#include "Scene.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/aruco.h>

#include "Ogre.h"
#include <OIS/OIS.h>

cv::VideoCapture TheVideoCapturer_left;
cv::Mat TheInputImage_left, TheInputImageUnd_left;

Ogre::PixelBox mPixelBox_left;
Ogre::TexturePtr mTexture_left;
aruco::CameraParameters CameraParams_left, CameraParamsUnd_left;

float TheMarkerSize = 0.025;

aruco::MarkerDetector TheMarkerDetector;
std::vector<aruco::Marker> TheMarkers;

Scene::Scene( Ogre::Root* root, OIS::Mouse* mouse, OIS::Keyboard* keyboard)
{

	TheVideoCapturer_left.open(0);

	CameraParams_left.readFromXMLFile("internal_camera.yml");
	CameraParamsUnd_left = CameraParams_left;
	CameraParamsUnd_left.Distorsion = cv::Mat::zeros(4, 1, CV_32F);

	mRoot = root;
	mMouse = mouse;
	mKeyboard = keyboard;
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	// Set up main Lighting and Shadows in Scene:
	//mSceneMgr->setAmbientLight( Ogre::ColourValue(0.1f,0.1f,0.1f) );
	//mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

	//mSceneMgr->setShadowFarDistance( 30 );

	//createRoom();
	createCameras();
}

Scene::~Scene()
{
	if (mSceneMgr) delete mSceneMgr;
}
/*
void Scene::createRoom()
{

	TheVideoCapturer.grab();
	TheVideoCapturer.retrieve(TheInputImage);
	cv::undistort(TheInputImage, TheInputImageUnd,
			CameraParams.CameraMatrix, CameraParams.Distorsion);

	//mRoomNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("RoomNode");

	Ogre::Camera *camera;
	Ogre::SceneNode* cameraNode;
	camera = mSceneMgr->createCamera("camera");
	camera->setNearClipDistance(0.01f);
	camera->setFarClipDistance(10.0f);
	camera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
	camera->setPosition(0, 0, 0);
	camera->lookAt(0, 0, 1);
	double pMatrix[16];
	CameraParamsUnd.OgreGetProjectionMatrix(CameraParamsUnd.CamSize, CameraParamsUnd.CamSize,
			pMatrix, 0.05, 10, false);
	Ogre::Matrix4 PM(pMatrix[0], pMatrix[1], pMatrix[2], pMatrix[3], pMatrix[4],
			pMatrix[5], pMatrix[6], pMatrix[7], pMatrix[8], pMatrix[9],
			pMatrix[10], pMatrix[11], pMatrix[12], pMatrix[13], pMatrix[14],
			pMatrix[15]);
	camera->setCustomProjectionMatrix(true, PM);
	camera->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);
	cameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("cameraNode");
	cameraNode->attachObject(camera);

	/// CREATE BACKGROUND FROM CAMERA IMAGE
	int width = CameraParamsUnd.CamSize.width;
	int height = CameraParamsUnd.CamSize.height;
	// create background camera image
	mPixelBox = Ogre::PixelBox(width, height, 1, Ogre::PF_R8G8B8, TheInputImageUnd.ptr<uchar>(0));
	// Create Texture
	mTexture = Ogre::TextureManager::getSingleton().createManual(
			"CameraTexture",
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
			Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

	//Create Camera Material
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
			"CameraMaterial",
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::Technique *technique = material->createTechnique();
	technique->createPass();
	material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
	material->getTechnique(0)->getPass(0)->createTextureUnitState(
			"CameraTexture");

	Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
	rect->setCorners(-1.0, 1.0, 1.0, -1.0);
	rect->setMaterial("CameraMaterial");

	// Render the background before everything else
	rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

	// Hacky, but we need to set the bounding box to something big, use infinite AAB to always stay visible
	Ogre::AxisAlignedBox aabInf;
	aabInf.setInfinite();
	rect->setBoundingBox(aabInf);

	// Attach background to the scene
	Ogre::SceneNode* cameraLeft = mSceneMgr->getRootSceneNode()->createChildSceneNode(
			"CameraLeft");
	cameraLeft->attachObject(rect);
}*/


void Scene::createCameras()
{
	// Left Camera
	TheVideoCapturer_left.grab();
	TheVideoCapturer_left.retrieve(TheInputImage_left);
	cv::undistort(TheInputImage_left, TheInputImageUnd_left,
			CameraParams_left.CameraMatrix, CameraParams_left.Distorsion);

	mCamLeft = mSceneMgr->createCamera("CameraLeft");

	mCamLeft->setNearClipDistance(0.01f);
	mCamLeft->setFarClipDistance(10.0f);
	mCamLeft->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
	mCamLeft->setPosition(0, 0, 0);
	mCamLeft->lookAt(0, 0, 1);
	double pMatrix[16];
	CameraParamsUnd_left.OgreGetProjectionMatrix(CameraParamsUnd_left.CamSize, CameraParamsUnd_left.CamSize,
			pMatrix, 0.05, 10, false);
	Ogre::Matrix4 PM(pMatrix[0], pMatrix[1], pMatrix[2], pMatrix[3], pMatrix[4],
			pMatrix[5], pMatrix[6], pMatrix[7], pMatrix[8], pMatrix[9],
			pMatrix[10], pMatrix[11], pMatrix[12], pMatrix[13], pMatrix[14],
			pMatrix[15]);
	mCamLeft->setCustomProjectionMatrix(true, PM);
	mCamLeft->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);

	/// CREATE BACKGROUND FROM CAMERA IMAGE
	int width = CameraParamsUnd_left.CamSize.width;
	int height = CameraParamsUnd_left.CamSize.height;
	// create background camera image
	mPixelBox_left = Ogre::PixelBox(width, height, 1, Ogre::PF_R8G8B8, TheInputImageUnd_left.ptr<uchar>(0));
	// Create Texture
	mTexture_left = Ogre::TextureManager::getSingleton().createManual(
			"CameraTexture_left",
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
			Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

	//Create Camera Material
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
			"CameraMaterial_left",
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::Technique *technique = material->createTechnique();
	technique->createPass();
	material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
	material->getTechnique(0)->getPass(0)->createTextureUnitState(
			"CameraTexture_left");

	Ogre::Rectangle2D* rect_left = new Ogre::Rectangle2D(true);
	rect_left->setCorners(-1.0, 1.0, 1.0, -1.0);
	rect_left->setMaterial("CameraMaterial_left");

	// Render the background before everything else
	rect_left->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

	// Hacky, but we need to set the bounding box to something big, use infinite AAB to always stay visible
	Ogre::AxisAlignedBox aabInf;
	aabInf.setInfinite();
	rect_left->setBoundingBox(aabInf);

	// Attach background to the scene
	Ogre::SceneNode* cameraLeft = mSceneMgr->getRootSceneNode()->createChildSceneNode("CameraLeft_scene");
	cameraLeft->attachObject(rect_left);

	////////////////////////////////
	//mCamLeft = mSceneMgr->createCamera("LeftCamera");
	mCamRight = mSceneMgr->createCamera("RightCamera");

	// Create a scene nodes which the cams will be attached to:
	mBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("BodyNode");
	mBodyTiltNode = mBodyNode->createChildSceneNode();
	mHeadNode = mBodyTiltNode->createChildSceneNode("HeadNode");
	mBodyNode->setFixedYawAxis( true );	// don't roll!

	//mHeadNode->attachObject(mCamLeft);
	mHeadNode->attachObject(mCamRight);

	// Position cameras according to interpupillary distance
	float dist = 0.05;
	/*if (mRift->isAttached())
	{
		dist = mRift->getStereoConfig().GetIPD();
	}*/
	mCamLeft->setPosition( -dist/2.0f, 0.0f, 0.0f );
	mCamRight->setPosition( dist/2.0f, 0.0f, 0.0f );

	mCamLeft->setFarClipDistance( 50 );
	mCamRight->setFarClipDistance( 50 );

	mCamLeft->setNearClipDistance( 0.001 );
	mCamRight->setNearClipDistance( 0.001 );


	mBodyNode->setPosition( 4.0, 1.5, 4.0 );
	//mBodyNode->lookAt( Ogre::Vector3::ZERO, Ogre::SceneNode::TS_WORLD );

	Ogre::Light* light = mSceneMgr->createLight();
	light->setType(Ogre::Light::LT_POINT);
	light->setCastShadows( false );
	light->setAttenuation( 65, 1.0, 0.07, 0.017 );
	light->setSpecularColour( .25, .25, .25 );
	light->setDiffuseColour( 0.35, 0.27, 0.23 );
	mBodyNode->attachObject(light);
}

void Scene::update( float dt )
{
	float forward = (mKeyboard->isKeyDown( OIS::KC_W ) ? 0 : 1) + (mKeyboard->isKeyDown( OIS::KC_S ) ? 0 : -1);
	float leftRight = (mKeyboard->isKeyDown( OIS::KC_A ) ? 0 : 1) + (mKeyboard->isKeyDown( OIS::KC_D ) ? 0 : -1);

	if( mKeyboard->isKeyDown( OIS::KC_LSHIFT ) )
	{
		forward *= 3;
		leftRight *= 3;
	}

	// Updating Left Eye
	TheVideoCapturer_left.grab();
	TheVideoCapturer_left.retrieve(TheInputImage_left);
	cv::undistort(TheInputImage_left, TheInputImageUnd_left,
			CameraParams_left.CameraMatrix, CameraParams_left.Distorsion);

	/// DETECT MARKERS
	TheMarkerDetector.detect(TheInputImageUnd_left, TheMarkers, CameraParamsUnd_left,
		TheMarkerSize);

	for (uint i = 0; i < TheMarkers.size(); i++)
		TheMarkers[i].draw(TheInputImageUnd_left, cv::Scalar(0, 0, 255), 1);

	mTexture_left->getBuffer()->blitFromMemory(mPixelBox_left);

	// Updating Right Eye

	Ogre::WindowEventUtilities::messagePump();

	Ogre::Vector3 dirX = mBodyTiltNode->_getDerivedOrientation()*Ogre::Vector3::UNIT_X;
	Ogre::Vector3 dirZ = mBodyTiltNode->_getDerivedOrientation()*Ogre::Vector3::UNIT_Z;

	mBodyNode->setPosition( mBodyNode->getPosition() + dirZ*forward*dt + dirX*leftRight*dt );
}

//////////////////////////////////////////////////////////////
// Handle Rift Input:
//////////////////////////////////////////////////////////////

void Scene::setRiftPose( Ogre::Quaternion orientation, Ogre::Vector3 pos )
{
	mHeadNode->setOrientation( orientation );
	mHeadNode->setPosition( pos );
}

void Scene::setIPD( float IPD )
{
	mCamLeft->setPosition( -IPD/2.0f, 0.0f, 0.0f );
	mCamRight->setPosition( IPD/2.0f, 0.0f, 0.0f );
}

//////////////////////////////////////////////////////////////
// Handle Input:
//////////////////////////////////////////////////////////////

bool Scene::keyPressed( const OIS::KeyEvent& e )
{
	return true;
}
bool Scene::keyReleased( const OIS::KeyEvent& e )
{
	return true;
}
bool Scene::mouseMoved( const OIS::MouseEvent& e )
{
	if( mMouse->getMouseState().buttonDown( OIS::MB_Left ) )
	{
		mBodyNode->yaw( Ogre::Degree( -0.3*e.state.X.rel ) );
		mBodyTiltNode->pitch( Ogre::Degree( -0.3*e.state.Y.rel ) );
	}
	return true;
}
bool Scene::mousePressed( const OIS::MouseEvent& e, OIS::MouseButtonID id )
{
	return true;
}
bool Scene::mouseReleased( const OIS::MouseEvent& e, OIS::MouseButtonID id )
{
	return true;
}

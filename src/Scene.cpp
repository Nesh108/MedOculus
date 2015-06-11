#include "Scene.h"

#include "Ogre.h"
#include <OIS/OIS.h>

float TheMarkerSize = 0.025;
aruco::MarkerDetector TheMarkerDetector;
std::vector<aruco::Marker> TheMarkers;

map<int, int> ARMarkersList;			// id - index
map<int, bool> ARMarkersState;		// id - visible?


Scene::Scene( Ogre::Root* root, OIS::Mouse* mouse, OIS::Keyboard* keyboard, Rift* rift)
{
	mRift = rift;
	mRoot = root;
	mMouse = mouse;
	mKeyboard = keyboard;
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	// Set up main Lighting and Shadows in Scene:
	//mSceneMgr->setAmbientLight( Ogre::ColourValue(0.1f,0.1f,0.1f) );
	//mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

	//mSceneMgr->setShadowFarDistance( 30 );

	//createRoom();
	loadARObjects();
	createCameras();
}

Scene::~Scene()
{
	if (mSceneMgr) delete mSceneMgr;
}


void Scene::loadARObjects()
{
	mRoomNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("RoomNode");

	Ogre::ResourceGroupManager::getSingleton().addResourceLocation("media",
				"FileSystem", "Popular");
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	for (int i = 0; i < 20; i++) {
		Ogre::String entityName = "ARO_"+ Ogre::StringConverter::toString(i);

		Ogre::Entity* ogreEntity = mSceneMgr->createEntity(entityName,
				"Cube.mesh");
		Ogre::Real offset = ogreEntity->getBoundingBox().getHalfSize().y;
		mARONodes[i] = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		Ogre::SceneNode *ogreNodeChild = mARONodes[i]->createChildSceneNode();
		ogreNodeChild->attachObject(ogreEntity);
		ogreNodeChild->rotate(Ogre::Vector3(1, 0, 0),
				Ogre::Radian(Ogre::Degree(90)));
		ogreNodeChild->translate(0, 0, offset, Ogre::Node::TS_PARENT);

		// TODO: Read from external source
		if(i == 0)
		{
			ARMarkersList[1009] = i;
			ARMarkersState[1009] = false;

			mARONodes[i]->setScale(1, 1, 1);
		}
		else if(i == 1)
		{
			ARMarkersList[724] = i;
			ARMarkersState[724] = false;

			mARONodes[i]->setScale(0.01, 0.01, 0.01);
		}
		else
			mARONodes[i]->setScale(0.5, 0.5, 0.5);

	}
	Ogre::Light* roomLight = mSceneMgr->createLight();
	roomLight->setType(Ogre::Light::LT_POINT);
	roomLight->setCastShadows( true );
	roomLight->setShadowFarDistance( 30 );
	roomLight->setAttenuation( 65, 1.0, 0.07, 0.017 );
	roomLight->setSpecularColour( .25, .25, .25 );
	roomLight->setDiffuseColour( 0.85, 0.76, 0.7 );

	roomLight->setPosition( 5, 5, 5 );

	mRoomNode->attachObject( roomLight );
}



void Scene::createCameras()
{
	mCamLeft = mSceneMgr->createCamera("LeftCamera");
	mCamRight = mSceneMgr->createCamera("RightCamera");

	// Create a scene nodes which the cams will be attached to:
	mBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("BodyNode");
	mBodyTiltNode = mBodyNode->createChildSceneNode();
	mHeadNode = mBodyTiltNode->createChildSceneNode("HeadNode");
	mBodyNode->setFixedYawAxis( true );	// don't roll!

	mHeadNode->attachObject(mCamLeft);
	mHeadNode->attachObject(mCamRight);

	// Position cameras according to interpupillary distance
	float dist = 0.05;
	/*if (mRift->isAttached())
	{
		dist = mRift->getStereoConfig().GetIPD();
	}*/
	mCamLeft->setPosition( -dist/2.0f, 0.0f, 0.0f );
	mCamRight->setPosition( dist/2.0f, 0.0f, 0.0f );

	// If possible, get the camera projection matricies given by the rift:
	/*if (mRift->isAttached())
	{
		mCamLeft->setCustomProjectionMatrix( true, mRift->getProjectionMatrix_Left() );
		mCamRight->setCustomProjectionMatrix( true, mRift->getProjectionMatrix_Right() );
	}*/
	mCamLeft->setFarClipDistance( 50 );
	mCamRight->setFarClipDistance( 50 );

	mCamLeft->setNearClipDistance( 0.001 );
	mCamRight->setNearClipDistance( 0.001 );

	/*mHeadLight = mSceneMgr->createLight();
	mHeadLight->setType(Ogre::Light::LT_POINT);
	mHeadLight->setCastShadows( true );
	mHeadLight->setShadowFarDistance( 30 );
	mHeadLight->setAttenuation( 65, 1.0, 0.07, 0.017 );
	mHeadLight->setSpecularColour( 1.0, 1.0, 1.0 );
	mHeadLight->setDiffuseColour( 1.0, 1.0, 1.0 );
	mHeadNode->attachObject( mHeadLight );*/

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

/*void Scene::createCameras()
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
	mCamLeft = mSceneMgr->createCamera("LeftCamera");
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
}*/

void Scene::update( float dt )
{

	cv::Mat TheInputImageUnd_left = mRift->getImageUndLeft();
	aruco::CameraParameters CameraParamsUnd_left = mRift->getCameraParamsUndLeft();

	/// DETECTING MARKERS on Left Camera
	TheMarkerDetector.detect(TheInputImageUnd_left, TheMarkers, CameraParamsUnd_left,
			TheMarkerSize);

	/// UPDATE SCENE
	uint i;
	double position[3], orientation[4];

	double markers_left[TheMarkers.size()];

	// show nodes for detected markers
	for (i = 0; i < TheMarkers.size(); i++) {
		ARMarkersState[TheMarkers[i].id] = false;
		TheMarkers[i].OgreGetPoseParameters(position, orientation);

		// TODO: add multiple ARObjects
		if (ARMarkersList.find(TheMarkers[i].id) != ARMarkersList.end()) {

			int index = ARMarkersList[TheMarkers[i].id];

			std::cout << "FOUND MARKER - Left Camera!! " << TheMarkers[i].id << " at index " << index << std::endl;

			mARONodes[index]->setPosition(position[0], position[1], position[2]);
			mARONodes[index]->setOrientation(orientation[0], orientation[1],
					orientation[2], orientation[3]);

			ARMarkersState[TheMarkers[i].id] = true;
			mARONodes[index]->setVisible(true);

		}
		else
			TheMarkers[i].draw(TheInputImageUnd_left, cv::Scalar(0, 0, 255), 1);

		// Store z coordinates
		markers_left[i] = position[2];

	}

	// Hide the other markers
	/*for(map<int,bool>::iterator it = ARMarkersState.begin(); it != ARMarkersState.end(); ++it) {
	  if(!it->second)
		  mARONodes[ARMarkersList[it->first]]->setVisible(false);
	}*/

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Find closest ID
	////////////////////////////////////////////////////////////////////////////////////////////////////
	float min = 1e5;
	int id_min = -1;
	for(uint j = 0 ; j < TheMarkers.size(); j++)
	{
		if(markers_left[j] < min)
		{
			min = markers_left[j];
			id_min = j;
		}
	}

	// Select closest marker
	if(id_min > -1)
	{
		cout << "#" << TheMarkers[id_min].id << " - Min Distance: " << min << "\n";
		mRift->selectMarker(TheInputImageUnd_left, TheMarkers[id_min], CameraParamsUnd_left);
	}

	mRift->setTextureLeft(mRift->getPixelBoxLeft());

	////////////////

	// Updating Right Eye
	if(mRift->countCameras() == 2){

		cv::Mat TheInputImageUnd_right = mRift->getImageUndRight();
		aruco::CameraParameters CameraParamsUnd_right = mRift->getCameraParamsUndRight();

		/// DETECTING MARKERS on Right Camera
		TheMarkerDetector.detect(TheInputImageUnd_right, TheMarkers, CameraParamsUnd_right,
				TheMarkerSize);

		double markers_right[TheMarkers.size()];

		// show nodes for detected markers
		for (i = 0; i < TheMarkers.size(); i++) {
			ARMarkersState[TheMarkers[i].id] = false;
			TheMarkers[i].OgreGetPoseParameters(position, orientation);

			// TODO: add multiple ARObjects
			if (ARMarkersList.find(TheMarkers[i].id) != ARMarkersList.end()) {

				int index = ARMarkersList[TheMarkers[i].id];

				std::cout << "FOUND MARKER - Right Camera!! " << TheMarkers[i].id << " at index " << index << std::endl;

				mARONodes[index]->setPosition(position[0], position[1], position[2]);
				mARONodes[index]->setOrientation(orientation[0], orientation[1],
						orientation[2], orientation[3]);

				ARMarkersState[TheMarkers[i].id] = true;
				mARONodes[index]->setVisible(true);

			}
			else
				TheMarkers[i].draw(TheInputImageUnd_right, cv::Scalar(0, 0, 255), 1);

			// Store z coordinates
			markers_right[i] = position[2];

		}

		// Hide the other markers
		/*for(map<int,bool>::iterator it = ARMarkersState.begin(); it != ARMarkersState.end(); ++it) {
		  if(!it->second)
			  mARONodes[ARMarkersList[it->first]]->setVisible(false);
		}*/

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// Find closest ID
		////////////////////////////////////////////////////////////////////////////////////////////////////
		float min = 1e5;
		int id_min = -1;
		for(uint j = 0 ; j < TheMarkers.size(); j++)
		{
			if(markers_right[j] < min)
			{
				min = markers_right[j];
				id_min = j;
			}
		}

		// Select closest marker
		if(id_min > -1)
		{
			cout << "#" << TheMarkers[id_min].id << " - Min Distance: " << min << "\n";
			mRift->selectMarker(TheInputImageUnd_right, TheMarkers[id_min], CameraParamsUnd_right);
		}

		mRift->setTextureRight(mRift->getPixelBoxRight());
	}


	float forward = (mKeyboard->isKeyDown( OIS::KC_W ) ? 0 : 1) + (mKeyboard->isKeyDown( OIS::KC_S ) ? 0 : -1);
	float leftRight = (mKeyboard->isKeyDown( OIS::KC_A ) ? 0 : 1) + (mKeyboard->isKeyDown( OIS::KC_D ) ? 0 : -1);

	if( mKeyboard->isKeyDown( OIS::KC_LSHIFT ) )
	{
		forward *= 3;
		leftRight *= 3;
	}

		//Ogre::WindowEventUtilities::messagePump();

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

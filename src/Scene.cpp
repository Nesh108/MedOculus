#include "Scene.h"

#include "Ogre.h"
#include <OIS/OIS.h>

#include "Leap.h"
#include <pthread.h>
#include <errno.h>

#include <curl/curl.h>
#include <Private.h>	// contains passwords and private data

#include "CURLplusplus.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <iostream>

using namespace std;
using namespace rapidjson;
using namespace Leap;

int SCALE_RATE = 80;
int ROTATION_RATE = 2;	// degrees

float TheMarkerSize = 0.025;
aruco::MarkerDetector TheMarkerDetector;
std::vector<aruco::Marker> TheMarkers;

map<int, int> ARMarkersList;			// id - index
map<int, bool> ARMarkersState;			// id - visible?
map<int, float> ARMarkersScale;			// id - scale
map<int, float> ARMarkersRotation;		// id - Xrotation
map<int, string> ARMarkersMesh;			// id - mesh

int errorT;
void *status;

pthread_t tid_leap;	//thread id

bool isRunning = false;
bool isLeapConnected = false;

int closest_marker = -1;
int selected_marker = -1;

bool USE_LEAP = true;
bool USE_ONLINE_DATA = true;

string SF_OAUTH_URL = "https://login.salesforce.com/services/oauth2/token";

class CURLplusplus;

class SampleListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);

  private:
};

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  isLeapConnected = true;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
	isLeapConnected = false;
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
	isLeapConnected = false;
  std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();

  HandList hands = frame.hands();

  // Get gestures
  const GestureList gestures = frame.gestures();
  for (int g = 0; g < gestures.count(); ++g) {
    Gesture gesture = gestures[g];

    switch (gesture.type()) {
      case Gesture::TYPE_CIRCLE:
      {
    	CircleGesture circle = gesture;
        std::string clockwiseness;

		float sweptAngle = 0;
		if (circle.state() != Gesture::STATE_START) {
		  CircleGesture previousUpdate = CircleGesture(controller.frame(1).gesture(circle.id()));
		  sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * PI;
		}

		cout << "Swept angle: " << sweptAngle * RAD_TO_DEG<< endl;
		if (circle.pointable().direction().angleTo(circle.normal()) <= PI/2) {
		  clockwiseness = "clockwise";

		  if(selected_marker != -1 && abs(sweptAngle * RAD_TO_DEG) > 5)
			  ARMarkersScale[selected_marker]  += ARMarkersScale[selected_marker]/SCALE_RATE;

		} else {
		  clockwiseness = "counterclockwise";

		  if(selected_marker != -1 && ARMarkersScale[selected_marker] - ARMarkersScale[selected_marker]/10 > 0  && abs(sweptAngle* RAD_TO_DEG) > 5)
			  ARMarkersScale[selected_marker] -= ARMarkersScale[selected_marker]/SCALE_RATE;

		}

		cout << "Changing scale ID #" << selected_marker << " to " << ARMarkersScale[selected_marker] <<  endl;  // Calculate angle swept since last frame

    	  std::cout << "GESTURE DETECTED: CIRCLE.\n";
        break;
      }
      case Gesture::TYPE_SWIPE:
      {
        SwipeGesture swipe = gesture;

        if(hands.leftmost().fingers().count() > 4){
			if(selected_marker != -1){
				if(swipe.direction()[0] >= 0)				// To the left
					ARMarkersRotation[selected_marker] += ROTATION_RATE;
				else if(swipe.direction()[0] < 0)		// To the right
					ARMarkersRotation[selected_marker] -= ROTATION_RATE;

				cout << "Current rotation: " << ARMarkersRotation[selected_marker] << endl;
			}
        }

        std::cout << "GESTURE DETECTED: SWIPE " << swipe.direction() << ".\n";;
        break;
      }
      case Gesture::TYPE_KEY_TAP:
      {
        KeyTapGesture tap = gesture;

			if(selected_marker == -1)
				selected_marker = closest_marker;
			else
				selected_marker = -1;

        cout << "Key Selected: " << selected_marker << endl;

        break;
      }
      case Gesture::TYPE_SCREEN_TAP:
      {
        ScreenTapGesture screentap = gesture;

			if(selected_marker == -1)
				selected_marker = closest_marker;
			else
				selected_marker = -1;
        break;
      }
      default:
        //std::cout << std::string(2, ' ')  << "Unknown gesture type." << std::endl;
        break;
    }
  }

}

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void SampleListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

Scene::Scene( Ogre::Root* root, OIS::Mouse* mouse, OIS::Keyboard* keyboard, Rift* rift)
{
	mRift = rift;
	mRoot = root;
	mMouse = mouse;
	mKeyboard = keyboard;
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	if(USE_ONLINE_DATA)
		initMarkersData();

	loadARObjects();
	createCameras();

	if(USE_LEAP)
		loadLeap();

}

Scene::~Scene()
{
	if (mSceneMgr) delete mSceneMgr;
	isRunning = false;
}

void Scene::initMarkersData(){

	fetchData();

}

void Scene::fetchData(){

	// Fetching stuff
	CURLplusplus client;
	string reply = client.Post(SF_OAUTH_URL, PASSWORD);
	cout << "Curl Got OAth: " << reply << endl;

	Document doc;
	doc.Parse(reply.c_str());
	Value& oath_token = doc["access_token"];
	Value& inst_url = doc["instance_url"];

	cout << "My access token: " << oath_token.GetString() << endl;

	char bh[200];
	sprintf(bh, "Authorization: Bearer %s",  oath_token.GetString());

	string bearer_header(bh);
	cout << "My bearer header: " <<  bearer_header << endl;

	char mUrl[400];
	sprintf(mUrl, "%s/services/data/v20.0/query?q=SELECT+(SELECT+Id,Name+FROM+Attachments),medicsales__argu__c,medicsales__rot_x__c,medicsales__rot_y__c,medicsales__rot_z__c,medicsales__scale_x__c,medicsales__scale_y__c,medicsales__scale_z__c,medicsales__marker_Id__c,Id+FROM+medicsales__dddObject__c",  inst_url.GetString());
	string fetch_url(mUrl);

	reply = client.Get(fetch_url, bearer_header);

	cout << "Curl Got Fetching: " << reply << endl;

	doc.Parse(reply.c_str());
	Value& records = doc["records"];
	Value& total_objects = doc["totalSize"];

	assert(records.IsArray());

	// Fetching data
	int count = 0;
	uint i;
	for (i = 0; i < total_objects.GetInt(); i++)
	{

		Value& marker_id = records[i]["medicsales__marker_Id__c"];
		Value& scale = records[i]["medicsales__scale_x__c"];
		Value& rot = records[i]["medicsales__rot_x__c"];
		Value& attachs = records[i]["Attachments"];
		attachs = attachs["records"];
		Value& id = attachs[0]["Id"];
		Value& name = attachs[0]["Name"];

		struct stat buf;
		char p[400];
		sprintf(p, "../media/%d-%s",  count, name.GetString());
		if (stat(p, &buf) == -1)
		{
			char mDataUrl[400];
			sprintf(mDataUrl, "%s/services/data/v20.0/sobjects/Attachment/%s/body",  inst_url.GetString(), id.GetString());
			string fetch_url(mDataUrl);

			printf("Fetching from: %s - records[%d] = %s - %s\n", fetch_url.c_str(), i, id.GetString(), name.GetString());
			reply = client.Get(fetch_url, bearer_header);

			ofstream myfile;

			char path[400];
			if(!hasEnding(name.GetString(),".material") && !hasEnding(name.GetString(),".skeleton"))
			{
				sprintf(path, "../media/%d-%s",  count, name.GetString());
				cout << "Writing to: " << path << endl;
				myfile.open (path);
				myfile << reply;
				myfile.close();

				int m_id = marker_id.GetDouble();
				ARMarkersList[m_id] = count;
				ARMarkersScale[m_id] = (float) scale.GetDouble();
				ARMarkersRotation[m_id] = (float) rot.GetDouble();

				cout << "Count: " << count << " - scale: " << ARMarkersScale[m_id] << " - rot: " << ARMarkersRotation[m_id] << endl;
				char mesh[400];
				sprintf(mesh, "%d-%s",  count, name.GetString());

				ARMarkersMesh[m_id] = mesh;

				cout << "Added mesh: " << mesh << " - id: " << m_id << endl;
				cout << "Count: " << count << endl;
				count++;
			}
			else
			{
				cout << "Skipped mesh" << endl;
				sprintf(path, "../media/%d-%s",  count-1, name.GetString());
				cout << "Writing to: " << path << endl;
				myfile.open (path);
				myfile << reply;
				myfile.close();

			}
		}
		else
			{
				cout << "Already existing: " << name.GetString() << endl;
				if(!hasEnding(name.GetString(),".material") && !hasEnding(name.GetString(),".skeleton"))
				{
					int m_id = marker_id.GetDouble();
					ARMarkersList[m_id] = count;
					ARMarkersScale[m_id] = (float) scale.GetDouble();
					ARMarkersRotation[m_id] = (float) rot.GetDouble();

					char mesh[400];
					sprintf(mesh, "%d-%s",  count, name.GetString());

					ARMarkersMesh[m_id] = mesh;

					count++;
				}
			}
	}

}

bool Scene::hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}


bool Scene::loadLeap(){

	isRunning = true;

	errorT = pthread_create(&tid_leap, NULL, leapThread, NULL);
	if(errorT){
		printf("pthread Leap is not created...\n");
		return false;
	}

	return true;
}

void *leapThread(void *arg){

	SampleListener listener;
	Controller controller;

	controller.addListener(listener);
	controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

	while(isRunning)
		;

	// Remove the sample listener when done
	controller.removeListener(listener);

	return (void *)0;

}
void Scene::loadARObjects()
{
	mRoomNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("RoomNode");

	Ogre::ResourceGroupManager::getSingleton().addResourceLocation("media",
				"FileSystem", "Popular");
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	for(map<int,int>::iterator it = ARMarkersList.begin(); it != ARMarkersList.end(); ++it) {

		int id = it->first;
		int i = it->second;

		string mesh_name = ARMarkersMesh[id];

		cout << "Adding id: " << id << " - mesh: " << mesh_name << " at index: " << i << endl;

		Ogre::String entityName = "ARO_"+ Ogre::StringConverter::toString(i);

		Ogre::Entity* ogreEntity = mSceneMgr->createEntity(entityName,
				mesh_name);
		Ogre::Real offset = ogreEntity->getBoundingBox().getHalfSize().y;
		mARONodes[i] = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		Ogre::SceneNode *ogreNodeChild = mARONodes[i]->createChildSceneNode();
		ogreNodeChild->attachObject(ogreEntity);
		ogreNodeChild->rotate(Ogre::Vector3(1, 0, 0),
				Ogre::Radian(Ogre::Degree(90)));
		ogreNodeChild->translate(0, 0, offset, Ogre::Node::TS_PARENT);

		ARMarkersList[id] = i;

		mARONodes[i]->setScale(ARMarkersScale[id], ARMarkersScale[id], ARMarkersScale[id]);
		mARONodes[i]->setVisible(false);

	}

	Ogre::Light* roomLight = mSceneMgr->createLight();
	roomLight->setType(Ogre::Light::LT_POINT);
	roomLight->setCastShadows( true );
	roomLight->setShadowFarDistance( 30 );
	roomLight->setAttenuation( 1000, 1.0, 0.07, 0.017 );
	roomLight->setSpecularColour( .25, .25, .25 );
	roomLight->setDiffuseColour( 0.85, 0.76, 0.7 );
	roomLight->setPowerScale(100);

	roomLight->setPosition( 5, 5, 5 );
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5,0.5,0.5));

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

	aruco::CameraParameters camParams_left;
	camParams_left.readFromXMLFile("camera_left.yml");

	double pMatrix[16];
	camParams_left.OgreGetProjectionMatrix(camParams_left.CamSize, camParams_left.CamSize,
				pMatrix, 0.05, 10, false);

	Ogre::Matrix4 PM_left(pMatrix[0], pMatrix[1], pMatrix[2], pMatrix[3], pMatrix[4],
			pMatrix[5], pMatrix[6], pMatrix[7], pMatrix[8], pMatrix[9],
			pMatrix[10], pMatrix[11], pMatrix[12], pMatrix[13], pMatrix[14],
			pMatrix[15]);
	mCamLeft->setCustomProjectionMatrix(true, PM_left);
	mCamLeft->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);

	///

	aruco::CameraParameters camParams_right;
	camParams_right.readFromXMLFile("camera_right.yml");

	camParams_right.OgreGetProjectionMatrix(camParams_right.CamSize, camParams_right.CamSize,
				pMatrix, 0.05, 10, false);

	Ogre::Matrix4 PM_right(pMatrix[0], pMatrix[1], pMatrix[2], pMatrix[3], pMatrix[4],
			pMatrix[5], pMatrix[6], pMatrix[7], pMatrix[8], pMatrix[9],
			pMatrix[10], pMatrix[11], pMatrix[12], pMatrix[13], pMatrix[14],
			pMatrix[15]);
	mCamRight->setCustomProjectionMatrix(true, PM_right);
	mCamRight->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);

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

	char leapStatus[400];
	cv::Point textPos(20,20);

	if(isLeapConnected)
		sprintf(leapStatus, "Leap is connected.");
	else
		sprintf(leapStatus, "Leap is not connected.");


	cv::Mat TheInputImageUnd_left = mRift->getImageUndLeft();
	aruco::CameraParameters CameraParamsUnd_left = mRift->getCameraParamsUndLeft();

	/// DETECTING MARKERS on Left Camera
	TheMarkerDetector.detect(TheInputImageUnd_left, TheMarkers, CameraParamsUnd_left,
			TheMarkerSize);

	cv::putText(TheInputImageUnd_left, leapStatus,textPos,cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(255), 1, CV_AA);

	/// UPDATE SCENE
	uint i;
	double position[3], orientation[4];

	double markers_left[TheMarkers.size()];

	// Clear marker state
	for(map<int,bool>::iterator it = ARMarkersState.begin(); it != ARMarkersState.end(); ++it) {
		it->second = false;
	}
	// show nodes for detected markers
	for (i = 0; i < TheMarkers.size(); i++) {
		TheMarkers[i].OgreGetPoseParameters(position, orientation);

		if (ARMarkersList.find(TheMarkers[i].id) != ARMarkersList.end()) {

			int index = ARMarkersList[TheMarkers[i].id];

			std::cout << "FOUND MARKER - Left Camera!! " << TheMarkers[i].id << " at index " << index << std::endl;

			mARONodes[index]->setPosition(position[0], position[1], position[2]);
			mARONodes[index]->setOrientation(orientation[0], orientation[1],
					orientation[2], orientation[3]);

			mARONodes[index]->rotate(Ogre::Vector3(0, 0, 1), Ogre::Radian(Ogre::Degree(ARMarkersRotation[TheMarkers[i].id])));
			mARONodes[index]->setScale(ARMarkersScale[TheMarkers[i].id],ARMarkersScale[TheMarkers[i].id],ARMarkersScale[TheMarkers[i].id]);

			ARMarkersState[TheMarkers[i].id] = true;
			mARONodes[index]->setVisible(true);

			if(selected_marker != -1 && selected_marker == TheMarkers[i].id)
				mRift->selectMarker(TheInputImageUnd_left, TheMarkers[i], CameraParamsUnd_left);

		}
		else
			TheMarkers[i].draw(TheInputImageUnd_left, cv::Scalar(0, 0, 255), 1);

		// Store z coordinates
		markers_left[i] = position[2];

	}

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
		closest_marker = TheMarkers[id_min].id;

	}

	mRift->setTextureLeft(mRift->getPixelBoxLeft());

	////////////////

	// Updating Right Eye
	if(mRift->countCameras() == 2){

		cv::Mat TheInputImageUnd_right = mRift->getImageUndRight();
		aruco::CameraParameters CameraParamsUnd_right = mRift->getCameraParamsUndRight();

		cv::putText(TheInputImageUnd_right, leapStatus,textPos,cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(255), 1, CV_AA);

		/// DETECTING MARKERS on Right Camera
		TheMarkerDetector.detect(TheInputImageUnd_right, TheMarkers, CameraParamsUnd_right,
				TheMarkerSize);

		double markers_right[TheMarkers.size()];

		// show nodes for detected markers
		for (i = 0; i < TheMarkers.size(); i++) {
			TheMarkers[i].OgreGetPoseParameters(position, orientation);

			if (ARMarkersList.find(TheMarkers[i].id) != ARMarkersList.end()) {

				int index = ARMarkersList[TheMarkers[i].id];

				std::cout << "FOUND MARKER - Right Camera!! " << TheMarkers[i].id << " at index " << index << std::endl;

				mARONodes[index]->setPosition(position[0], position[1], position[2]);
				mARONodes[index]->setOrientation(orientation[0], orientation[1],
									orientation[2], orientation[3]);

				mARONodes[index]->rotate(Ogre::Vector3(0, 0, 1), Ogre::Radian(Ogre::Degree(ARMarkersRotation[TheMarkers[i].id])));

				mARONodes[index]->setScale(ARMarkersScale[TheMarkers[i].id],ARMarkersScale[TheMarkers[i].id],ARMarkersScale[TheMarkers[i].id]);

				ARMarkersState[TheMarkers[i].id] = true;
				mARONodes[index]->setVisible(true);

				if(selected_marker != -1 && selected_marker == TheMarkers[i].id)
					mRift->selectMarker(TheInputImageUnd_right,  TheMarkers[i], CameraParamsUnd_right);


			}
			else
				TheMarkers[i].draw(TheInputImageUnd_right, cv::Scalar(0, 0, 255), 1);

			// Store z coordinates
			markers_right[i] = position[2];

		}

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
			closest_marker = TheMarkers[id_min].id;

		}

		mRift->setTextureRight(mRift->getPixelBoxRight());
	}

	// Hide the other markers
	for(map<int,bool>::iterator it = ARMarkersState.begin(); it != ARMarkersState.end(); ++it) {
	  if(!it->second)
		  mARONodes[ARMarkersList[it->first]]->setVisible(false);

	}

	float forward = (mKeyboard->isKeyDown( OIS::KC_W ) ? 0 : 1) + (mKeyboard->isKeyDown( OIS::KC_S ) ? 0 : -1);
	float leftRight = (mKeyboard->isKeyDown( OIS::KC_A ) ? 0 : 1) + (mKeyboard->isKeyDown( OIS::KC_D ) ? 0 : -1);

	if( mKeyboard->isKeyDown( OIS::KC_LSHIFT ) )
	{
		forward *= 3;
		leftRight *= 3;
	}

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

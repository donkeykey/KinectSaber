#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxNetwork.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxKFW2::Device kinect;

	ofxUDPManager udpConnection;

	ofVec2f rightHandPos;
	ofVec3f rightHandOriginalPos;
	ofVec3f kensakiOriginalPos;
	ofVec3f kensakiOriginalPosOld;
	ofVec3f kensakiSpeedVec;
	ICoordinateMapper* m_pCoordinateMapper;

	float saberScale;

	float pitch, roll, yaw;
	float pitchAdp, rollAdp, yawAdp;

	ofVec2f testPos1;
	ofVec2f testPos2;

	ofSoundPlayer saberUsualSound;
	ofSoundPlayer saberOnSound;
	ofSoundPlayer saberOffSound;
	ofSoundPlayer hitSounds[4];
	double soundSpeed;
	bool saberSwitch;

	ofFloatPixels depthToWorldTable;

private:
	void drawSaber();
	vector<string> split(const string &str, char delim);
	void calcSaberPoint(ofVec3f rightHandPos, ofVec3f saberVec, float scale);
	ofxKinectForWindows2::Data::Joint fowardJoint;
	ofVec2f kenmotoPos1, kenmotoPos2, kenmotoPos3, kensakiPos1, kensakiPos2, kensakiPos3;

	float depthWidth;
	float depthHeight;
	float depthSize;
	unsigned short* depthPixel;
	float xScale, yScale;
	vector<ofVec3f> saberCircleArr;
	int hitFlag;

	ofImage deathStars[10];
	ofImage sparks[10];
	ofVec3f deathStarOriginalPos[10];
	ofVec2f deathStarPos[10];
	bool deathDeathFlag[10], deathStarVisible[10];
	int sparksFlag[10];
	int counter, deathIndex;
	float deathStarDepth[10];
	int deathStarSize[10];
	bool fowardFlag[10];
	int point;
	int sec, sec_tmp;
	ofTrueTypeFont font;

};
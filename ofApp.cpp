#include "ofApp.h"

int previewWidth = 960;
int previewHeight = 540;

//int previewWidth = 512;
//int previewHeight = 424;


//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	kinect.getSensor()->get_CoordinateMapper(&m_pCoordinateMapper);

	ofSetWindowShape(previewWidth, previewHeight);

	udpConnection.Create();
	udpConnection.Connect("192.168.1.115", 8060);
	udpConnection.Bind(8059);
	udpConnection.SetNonBlocking(true);

	roll, pitch, yaw = 0.0;
	rollAdp, pitchAdp, yawAdp = 0.0;

	// Sound
	saberUsualSound.load("sound/Saberftn.wav");
	saberUsualSound.setLoop(true);
	//saberUsualSound.play();
	saberUsualSound.setSpeed(1.0f);

	saberOnSound.load("sound/fx4.wav");
	saberOnSound.setVolume(0.1f);

	saberOffSound.load("sound/fx5.wav");
	saberOffSound.setVolume(0.1f);

	for (int i = 0; i < 4; i++) {
		int num = i + 1;
		string str = "sound/lasrhit";
		str += to_string(num);
		hitSounds[i].load(str +  ".WAV");
		hitSounds[i].setVolume(0.1f);
	}


	kensakiOriginalPosOld = { 0, 0, 0 };
	soundSpeed = 0.0;

	saberSwitch = false;

	saberScale = 0.0f;

	xScale = 1980 / previewWidth;
	yScale = 1080 / previewHeight;

	saberCircleArr = { {} };

	hitFlag = 0;

	for (int i = 0; i < 10; i++) {
		deathStars[i].loadImage("death_star.gif");
		sparks[i].loadImage("sparks.gif");
		deathStarOriginalPos[i] = { 0, 0, 0 };
		deathDeathFlag[i] = true;
		deathStarVisible[i] = true;
		deathStarSize[i] = 20;
		fowardFlag[i] = true;

	}
	counter = 0;
	deathIndex = 0;

	point = 0;
	sec = 0;
	font.loadFont("arial.ttf", 52);

}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	char udpMessage[256];
	char delim = ':';
	udpConnection.Receive(udpMessage, 256);

	string message = udpMessage;
	//cout << "message : " << message << endl;

	vector<string> splitStr = split(message, delim);
	if (splitStr[0] == "roll") {
		roll = stof(splitStr[1]);
		//roll = -roll;
	}
	if (splitStr[0] == "pitch") {
		pitch = stof(splitStr[1]);
	}
	if (splitStr[0] == "yaw") {
		yaw = stof(splitStr[1]);
		yaw = -yaw;
	}
	if (splitStr[0] == "switch") {
		cout << splitStr[1] << endl;
		if (splitStr[1] == "1") {
			saberSwitch = true;
			saberOnSound.play();
			saberUsualSound.play();
		}
		else {
			saberSwitch = false;
			saberOffSound.play();
			saberUsualSound.stop();
		}
	}
	if (saberSwitch) {
		if (saberScale <= 0.8f) {
			saberScale += 0.05f;
		}
	} else {
		if (saberScale >= 0.0f) {
			saberScale -= 0.05f;
		}
	}
	//cout << "saberScale : " << saberScale << endl;

	//--
	//Getting joint positions (skeleton tracking)
	//--
	//
	{
		auto bodies = kinect.getBodySource()->getBodies();
		ofVec3f posRHand = { 999, 999, 999 };
		ofVec3f posRElb = { 999, 999, 999 };
		for (auto body : bodies) {
			for (auto joint : body.joints) {
				//now do something with the joints
				JointType type = joint.second.getType();
				if (type == JointType_HandRight) {
					if (posRHand.z > joint.second.getPosition().z) {
						posRHand = joint.second.getPosition();
						fowardJoint = joint.second;
					}
					rightHandPos = fowardJoint.getProjected(m_pCoordinateMapper, ofxKinectForWindows2::ColorCamera);
					rightHandOriginalPos = posRHand;
				}
				/*
				if (type == JointType_ElbowRight) {
					if (posRElb.z > joint.second.getPosition().z) {
						posRElb = joint.second.getPosition();
						ofQuaternion rightElbQuat = joint.second.getOrientation();
						string message = "rightElb:";
						message += to_string(rightElbQuat.getEuler().x) + ",";
						message += to_string(rightElbQuat.getEuler().y) + ",";
						message += to_string(rightElbQuat.getEuler().z);
						udpConnection.Send(message.c_str(), message.length());
					}
				}
				*/
			}
		}
	}

	// 剣のベクトルを求める
	float saberVecX = cos((yaw - yawAdp) * PI / 180) * cos((pitch - pitchAdp) * PI / 180);
	float saberVecZ = sin((yaw - yawAdp) * PI / 180) * cos((pitch - pitchAdp) * PI / 180);
	float saberVecY = sin((pitch - pitchAdp) * PI / 180);

	//	基準点調整
	if (splitStr[0] == "reset") {
		//cout << "reset!" << endl;
		pitchAdp = pitch;
		yawAdp = yaw + ((90 * 180) / PI);
	}

	// ライトセーバー描画
	//if (saberScale >= 0) {
		ofVec3f saberVec = { saberVecX, saberVecY, saberVecZ };
		calcSaberPoint(rightHandOriginalPos, saberVec, saberScale);
	//}
	// 速さ計算
	kensakiSpeedVec = kensakiOriginalPosOld - kensakiOriginalPos;
	double speed = ((pow(kensakiSpeedVec.x, 2.0) + pow(kensakiSpeedVec.y, 2.0) + pow(kensakiSpeedVec.z, 2.0)) * 5) + 1;
	if (speed > 2.0) {
		speed = 2.0;
	}
	if (speed > soundSpeed) {
		soundSpeed = speed;
	}
	else {
		soundSpeed -= 0.03;
	}
	//cout << "soundSpeed : " << soundSpeed << endl;
	saberUsualSound.setSpeed(soundSpeed);

	kensakiOriginalPosOld = kensakiOriginalPos;

	

	//--
	//Getting Depth Source
	//--
	//
	if (kinect.isFrameNew()) {
		auto depth = kinect.getDepthSource();

		{
			depth->getDepthToWorldTable(depthToWorldTable);
		}

		{
			depthWidth = depth->getWidth();
			depthHeight = depth->getHeight();
			depthSize = depthWidth * depthHeight;
			depthPixel = depth->getPixels().getData();
		}
	}

	// 敵表示
	/*
	for (int i = 0; i < 10; i++) {
		CameraSpacePoint cameraSpacePoint;
		cameraSpacePoint.X = deathStarOriginalPos[i].x;
		cameraSpacePoint.Y = deathStarOriginalPos[i].y;
		cameraSpacePoint.Z = deathStarOriginalPos[i].z;
		ColorSpacePoint projected = { 0 };
		HRESULT hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
		deathStarPos[i].x = projected.X / xScale;
		deathStarPos[i].y = projected.Y / yScale;
	}
	*/

	if (counter > 100) {
		deathStars[deathIndex % 10].loadImage("death_star.gif");
		sparks[deathIndex % 10].loadImage("sparks.gif");

		deathDeathFlag[deathIndex % 10] = false;
		deathStarOriginalPos[deathIndex % 10] = { (ofRandom(200) / 100.0f) - 1.0f,(ofRandom(200) / 100.0f) - 1.0f, 1.0f + ofRandom(200) / 100.0f };
		deathStarSize[deathIndex % 10] = 20 + (3 - deathStarOriginalPos[deathIndex % 10].z) * 30;
		counter = 0;
		//cout << "xxx:" << deathStarOriginalPos[deathIndex % 10].x << " yyy:" << deathStarOriginalPos[deathIndex % 10].y << " zzz:" << deathStarOriginalPos[deathIndex % 10].z << endl;
		CameraSpacePoint cameraSpacePoint;
		cameraSpacePoint.X = deathStarOriginalPos[deathIndex % 10].x;
		cameraSpacePoint.Y = deathStarOriginalPos[deathIndex % 10].y;
		cameraSpacePoint.Z = deathStarOriginalPos[deathIndex % 10].z;
		ColorSpacePoint projected = { 0 };
		HRESULT hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
		deathStarPos[deathIndex % 10].x = projected.X / xScale;
		deathStarPos[deathIndex % 10].y = projected.Y / yScale;
		deathIndex++;
		sec = ofGetSeconds();
		if (sec < sec_tmp) {
			point = 0;
			for (int i = 0; i < 10; i++) {
				deathDeathFlag[i] = true;
			}
		}
		sec_tmp = sec;
	}
	counter++;
	
	for (int i = 0; i < 10; i++) {
		// depth計算
		CameraSpacePoint cameraSpacePoint;
		cameraSpacePoint.X = deathStarOriginalPos[i].x;
		cameraSpacePoint.Y = deathStarOriginalPos[i].y;
		cameraSpacePoint.Z = deathStarOriginalPos[i].z;

		DepthSpacePoint dprojected = { 0 };
		HRESULT hr = m_pCoordinateMapper->MapCameraPointToDepthSpace(cameraSpacePoint, &dprojected);
		int mappedX = dprojected.X;
		int mappedY = dprojected.Y;
		int index = (mappedY * depthWidth) + mappedX;
		//deathStarDepth[i] = depthPixel[index];
		if (0 <= index && index < 512 * 424) {
			/*
			if (i == 0) {
				cout << "depth:" << depthPixel[index] / 7000.0f << endl;
				cout << "originalposz:" << deathStarOriginalPos[i].z << endl;
			}
			*/
			deathStarDepth[i] = depthPixel[index];
		}
		sparksFlag[i]--;
	}
	
}

//--------------------------------------------------------------
void ofApp::draw() {
	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (previewHeight - colorHeight) / 2.0;
	kinect.getColorSource()->draw(0, 0, previewWidth, previewHeight);

	//death star
	ofSetColor(255, 255, 255);
	for (int i = 0; i < 10; i++) {
		int size = deathStarSize[i];
		if (!deathDeathFlag[i] && deathStarVisible[i]) {
			deathStars[i].resize(size, size);
			deathStars[i].draw(deathStarPos[i].x, deathStarPos[i].y);
			//cout << "x:" << deathStarPos[i].x << " y:" << deathStarPos[i].y << endl;
		}
		if (sparksFlag[i] > 0) {
			sparks[i].resize(size * 1.5, size * 1.5);
			sparks[i].draw(deathStarPos[i].x, deathStarPos[i].y);
		}
	}

	// ライトセーバー描画
	if (saberScale >= 0) {
		drawSaber();
	}

	string text = "Time : " + ofToString(sec) + "\n" + "Score : " + ofToString(point);
	font.drawString(text, 20, previewHeight - 100);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	int mappedX = x * (depthWidth / previewWidth);
	int mappedY = y * (depthHeight / previewHeight);
	//cout << "depthWidth : " << depthWidth << " & depthHeight : " << depthHeight << endl;
	//cout << "touched : x=" << x << ",y=" << y << endl;
	//cout << "mapped : x=" << mappedX << ",y=" << mappedY << endl;
	int index = (mappedY * depthWidth) + mappedX;
	//cout << "depth : " << depthPixel[index] << endl;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::drawSaber() {
	for (int i = 0; i < saberCircleArr.size(); i++) {
		//for (int j = 0; j < 10; j++) {
			//if (fowardFlag[j])
			ofSetColor(0, 60, 255, 20);
			ofCircle(saberCircleArr[i].x, saberCircleArr[i].y, saberCircleArr[i].z);
			ofSetColor(0, 60, 255, 255);
			ofCircle(saberCircleArr[i].x, saberCircleArr[i].y, saberCircleArr[i].z / 2.0f);
		//}
	}

	/*
	fbo.end();
	processFbo.begin();
	fbo.getTextureReference().bind();
	glBegin(GL_QUADS);
	glTexCoord2f(10, 0), glVertex2d(0, 0);
	glTexCoord2f(previewWidth, 0), glVertex2d(ofGetWidth(), 0);
	glTexCoord2f(previewWidth, previewHeight), glVertex2d(previewWidth, previewHeight);
	glTexCoord2f(10, previewHeight), glVertex2d(0, previewHeight);
	glEnd();
	fbo.getTextureReference().unbind();
	processFbo.end();
	fbo.draw(0, 0);
	*/
	/*
	path.moveTo(100, 100);
	path.lineTo(200, 100);
	path.lineTo(200, 200);
	path.lineTo(100, 200);
	path.lineTo(100, 100);
	path.draw();
	ofSetColor(255, 255, 255);
	for (int i = 0; i < 10; i++) {

	}
	/*
	ofSetLineWidth(20);
	ofDrawLine(rightHandPos.x / xScale, rightHandPos.y / yScale, kensakiPos1.x / xScale, kensakiPos1.y / yScale);
	ofDrawLine(kenmotoPos3.x / xScale, kenmotoPos3.y / yScale, kenmotoPos1.x / xScale, kenmotoPos1.y / yScale);
	ofDrawLine(kenmotoPos3.x / xScale, kenmotoPos3.y / yScale, kenmotoPos2.x / xScale, kenmotoPos2.y / yScale);
	ofDrawLine(kenmotoPos1.x / xScale, kenmotoPos1.y / yScale, kensakiPos2.x / xScale, kensakiPos2.y / yScale);
	ofDrawLine(kenmotoPos2.x / xScale, kenmotoPos2.y / yScale, kensakiPos3.x / xScale, kensakiPos3.y / yScale);
	ofDrawLine(kensakiPos1.x / xScale, kensakiPos1.y / yScale, kensakiPos3.x / xScale, kensakiPos3.y / yScale);
	ofDrawLine(kensakiPos1.x / xScale, kensakiPos1.y / yScale, kensakiPos2.x / xScale, kensakiPos2.y / yScale);
	*/
	ofSetColor(255, 255, 255);
}

void ofApp::calcSaberPoint(ofVec3f rHandPos, ofVec3f saberVec, float scale) {
	saberVec.normalize();
	ofVec3f saberLengthVec = saberVec * scale;

	// 剣先
	kensakiOriginalPos = rightHandOriginalPos + saberLengthVec;
	CameraSpacePoint cameraSpacePoint;
	cameraSpacePoint.X = kensakiOriginalPos.x;
	cameraSpacePoint.Y = kensakiOriginalPos.y;
	cameraSpacePoint.Z = kensakiOriginalPos.z;
	ColorSpacePoint projected = { 0 };
	HRESULT hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
	kensakiPos1.x = projected.X;
	kensakiPos1.y = projected.Y;

	int section = 300;
	vector<ofVec3f> cirArr = { {} };
	for (int i = 0; i < section; i++) {
		cirArr.push_back({ (rightHandOriginalPos.x + ((kensakiOriginalPos.x - rightHandOriginalPos.x) / section) * i), (rightHandOriginalPos.y + ((kensakiOriginalPos.y - rightHandOriginalPos.y) / section) * i), (rightHandOriginalPos.z + ((kensakiOriginalPos.z - rightHandOriginalPos.z) / section) * i) });
	}

	// デススターをdepth座標に変換
	ofVec2f deathDepthPos[10];
	for (int i = 0; i < 10; i++) {
		CameraSpacePoint csp;
		csp.X = deathStarOriginalPos[i].x;
		csp.Y = deathStarOriginalPos[i].y;
		csp.Z = deathStarOriginalPos[i].z;
		DepthSpacePoint dprojected = { 0 };
		HRESULT hr = m_pCoordinateMapper->MapCameraPointToDepthSpace(csp, &dprojected);
		deathDepthPos[i].x = dprojected.X;
		deathDepthPos[i].y = dprojected.Y;
	}

	saberCircleArr = { {} };
	for (int i = 0; i < cirArr.size(); i++) {
		//cout//
		if (i == cirArr.size() - 1) {
			//cout << "x:" << cirArr[i].x << ",y:" << cirArr[i].y << ",z:" << cirArr[i].z << endl;
		}
		//cout//
		CameraSpacePoint cameraSpacePoint;
		cameraSpacePoint.X = cirArr[i].x;
		cameraSpacePoint.Y = cirArr[i].y;
		cameraSpacePoint.Z = cirArr[i].z;
		ColorSpacePoint cprojected = { 0 };
		HRESULT hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &cprojected);
		float circleX = cprojected.X / xScale;
		float circleY = cprojected.Y / yScale;

		// depth計算
		DepthSpacePoint dprojected = { 0 };
		hr = m_pCoordinateMapper->MapCameraPointToDepthSpace(cameraSpacePoint, &dprojected);
		//cout << "depth x :" << dprojected.X << "and y : " << dprojected.Y << endl;
		////////////
		
		//int mappedX = circleX * depthWidth / previewWidth;
		//int mappedY = circleY * depthHeight / previewHeight;
		int mappedX = dprojected.X;
		int mappedY = dprojected.Y;
		int index = (mappedY * depthWidth) + mappedX;
		if (0 <= index && index < 512 * 424) {
			if (depthPixel[index] / 700 > cirArr[i].z && depthPixel[index] != 0) { // ライトセーバーが物体の裏に回ってるかどうか
				// ライトセーバーがデススターの裏に回ってるかどうか
				//for (int j = 0; j < 10; j++) {
					//if (!deathDeathFlag[j]) {
						//cout << "mappedX:" << mappedX << " deathdapthposx:" << deathDepthPos[j].x << endl;
						//cout << "cirarrz:" << cirArr[i].z << " posz:" << deathStarOriginalPos[j].z << endl;
						//if (cirArr[i].z > deathStarOriginalPos[j].z && deathDepthPos[j].x - deathStarSize[j] / 2 < mappedX && deathDepthPos[j].x > mappedX && deathDepthPos[j].y < mappedY && deathDepthPos[j].y + deathStarSize[j] / 2 > mappedY) {
						//if (deathDepthPos[j].x - deathStarSize[j] / 2 < mappedX && deathDepthPos[j].x > mappedX && deathDepthPos[j].y < mappedY && deathDepthPos[j].y + deathStarSize[j] / 2 > mappedY) {
						//if (deathStarOriginalPos[j].x - deathStarSize[j] / 170.0f < cirArr[i].x && deathStarOriginalPos[j].x > cirArr[i].x && deathStarOriginalPos[j].y < cirArr[i].y && deathStarOriginalPos[j].y + deathStarSize[j] / 170.0f > cirArr[i].y) {
						/*
						if (cirArr[i].z > deathStarOriginalPos[j].z) {
							fowardFlag[i] = false;
						}
						*/
					//}
				//}
				//if (fowardFlag) {
					saberCircleArr.push_back({ circleX, circleY, (3 - (rightHandOriginalPos.z + ((kensakiOriginalPos.z - rightHandOriginalPos.z) / section) * i)) * 6 });
				//}
				// ライトセーバーにデススターが当たってるかどうか
				float adp = 0.1f;
				for (int j = 0; j < 10; j++) {
					if (deathStarOriginalPos[j].x < cirArr[i].x + adp && deathStarOriginalPos[j].x > cirArr[i].x - adp && deathStarOriginalPos[j].y < cirArr[i].y + adp && deathStarOriginalPos[j].y > cirArr[i].y - adp && deathStarOriginalPos[j].z < cirArr[i].z + adp*5 && deathStarOriginalPos[j].z > cirArr[i].z - adp*5) {
						if (!deathDeathFlag[j]) {
							hitSounds[(int)ofRandom(3)].play();
							deathDeathFlag[j] = true;
							point += 10;
							sparksFlag[j] = 30;
							//if (hitFlag == 4) {
							//}
						}
					}
				}
				// 先端がどこかにあたってるかどうか
				//if (i == cirArr.size() - 1) {
				//	hitFlag = 0;
				//}
			} else {
				if (i == cirArr.size() - 1) {
					/*
					if (hitFlag == 4) {
						hitSounds[(int)ofRandom(3)].play();
					}
					*/
					//hitFlag++;
				}

			}
		} else {
			saberCircleArr.push_back({ circleX, circleY, (3 - (rightHandOriginalPos.z + ((kensakiOriginalPos.z - rightHandOriginalPos.z) / section) * i)) * 6 });
		}
		// deathStar
		for (int j = 0; j < 10; j++) {
			/*
			if ((saberScale <= 0.5) || (deathStarDepth[j] / 700.0f > deathStarOriginalPos[j].z || cirArr[i].z > deathStarOriginalPos[j].z && (cirArr[i].y < deathStarOriginalPos[j].y - 0.02f || cirArr[i].y > deathStarOriginalPos[j].y + 0.02f || cirArr[i].x < deathStarOriginalPos[j].x - 0.02f || cirArr[i].x > deathStarOriginalPos[j].x - 0.02f))) {
				deathStarVisible[j] = true;
			}
			else {
				deathStarVisible[j] = false;
			}
			*/
			//if (deathStarDepth[j] / 700.0f < deathStarOriginalPos[j].z || (cirArr[i].z < deathStarOriginalPos[j].z && (deathStarOriginalPos[j].x - 0.2f < cirArr[i].x && deathStarOriginalPos[j].x + 0.2f > cirArr[i].x && deathStarOriginalPos[j].y - 0.2f < cirArr[i].y && deathStarOriginalPos[j].y + 0.2f > cirArr[i].y))) {
			if (deathStarDepth[j] / 700.0f < deathStarOriginalPos[j].z) {
				deathStarVisible[j] = false;
			} else {
				deathStarVisible[j] = true;
			}
		}

	}

	int mappedX = saberCircleArr[saberCircleArr.size() - 1].x * depthWidth / previewWidth;
	int mappedY = saberCircleArr[saberCircleArr.size() - 1].y * depthHeight / previewHeight;
	//cout << "x:" << mappedX << "  &  y :" << mappedY << endl;

	//cout << "depthWidth : " << depthWidth << " & depthHeight : " << depthHeight << endl;
	//cout << "touched : x=" << x << ",y=" << y << endl;
	//cout << "mapped : x=" << mappedX << ",y=" << mappedY << endl;
	int index = (mappedY * depthWidth) + mappedX;
	if (0 <= index && index < 512 * 424) {
		//cout << "depth : " << depthPixel[index] << endl;
	}
	// (3 - (rightHandOriginalPos.z + ((kensakiOriginalPos.z - rightHandOriginalPos.z) / section) * i)) * 6 

	/*
	// 剣元Vector
	ofVec3f kenmotoVec = { -1 * (saberVec.y / saberVec.x), 1, 0 };
	kenmotoVec.normalize();
	kenmotoVec = kenmotoVec;

	// 剣元1
	ofVec3f kenmoto1 = rightHandOriginalPos + (saberVec * 0.08) + (kenmotoVec * 0.01f);
	//cout << kenmoto1 << endl;
	cameraSpacePoint.X = kenmoto1.x;
	cameraSpacePoint.Y = kenmoto1.y;
	cameraSpacePoint.Z = kenmoto1.z;
	projected = { 0 };
	hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
	kenmotoPos1 = { projected.X, projected.Y };
	
	// 剣元2
	ofVec3f kenmoto2 = rightHandOriginalPos + (saberVec * 0.08) + kenmotoVec * (-0.01f);
	cameraSpacePoint.X = kenmoto2.x;
	cameraSpacePoint.Y = kenmoto2.y;
	cameraSpacePoint.Z = kenmoto2.z;
	projected = { 0 };
	hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
	kenmotoPos2 = { projected.X, projected.Y };

	// 剣元3
	ofVec3f kenmoto3 = rightHandOriginalPos + (saberVec * 0.08);
	cameraSpacePoint.X = kenmoto3.x;
	cameraSpacePoint.Y = kenmoto3.y;
	cameraSpacePoint.Z = kenmoto3.z;
	projected = { 0 };
	hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
	kenmotoPos3 = { projected.X, projected.Y };

	// 剣先2
	ofVec3f kensaki2 = kenmoto1 + saberVec * 0.6;
	cameraSpacePoint.X = kensaki2.x;
	cameraSpacePoint.Y = kensaki2.y;
	cameraSpacePoint.Z = kensaki2.z;
	projected = { 0 };
	hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
	kensakiPos2.x = projected.X;
	kensakiPos2.y = projected.Y;

	// 剣先3
	ofVec3f kensaki3 = kenmoto2 + saberVec * 0.6;
	cameraSpacePoint.X = kensaki3.x;
	cameraSpacePoint.Y = kensaki3.y;
	cameraSpacePoint.Z = kensaki3.z;
	projected = { 0 };
	hr = m_pCoordinateMapper->MapCameraPointToColorSpace(cameraSpacePoint, &projected);
	kensakiPos3.x = projected.X;
	kensakiPos3.y = projected.Y;
	*/

}

 vector<string> ofApp::split(const string &s, char delim) {
	 vector<string> elms;
	 size_t offset = 0;
	 while (true) {
		 size_t next = s.find_first_of(delim, offset);
		 if (next == string::npos) {
			 elms.push_back(s.substr(offset));
			 return elms;
		 }
		 elms.push_back(s.substr(offset, next - offset));
		 offset = next + 1;
	 }
}
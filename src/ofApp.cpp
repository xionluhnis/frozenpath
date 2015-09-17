#include "ofApp.h"
#include <cstdlib>
#include <iostream>

//--------------------------------------------------------------
void ofApp::setup()
{
    std::cout << "Initializing OpenGL Context.\n";
    ofSetFrameRate(30);
    // ofEnableAntiAliasing();
    ofEnableDepthTest(); //make sure we test depth for 3d
    ofSetVerticalSync(true);
    ofEnableLighting();
    // ofEnableAlphaBlending();
    // ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofEnableSmoothing();

    // light the scene to show off why normals are important
    light.enable();
    light.setPointLight();
    light.setPosition(0, 0, 300);

    std::cout << "Initializing Depth Sensor with OpenNI2.\n";
    pcGrabber = new pcl::io::OpenNI2Grabber();
    
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = 
    	boost::bind(&ofApp::updateCloud, this, _1);
    pcGrabber->registerCallback(f);
    pcGrabber->start();
    
    running = true;
}

//--------------------------------------------------------------
void ofApp::update()
{
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofEnableLighting();
    ofSetColor(255, 255, 255, 255);

    // draw background
    ofColor centerColor = ofColor(85, 78, 68);
    ofColor edgeColor(0, 0, 0);
    ofBackgroundGradient(centerColor, edgeColor, OF_GRADIENT_CIRCULAR);

    cam.begin();
    // display point cloud
    ofMeshPtr pc = pointCloud; // create a copy so that it may be replaced while being used (from OpenNI update)
    if(pc) {
    		ofSetColor(255, 255, 255, 150);
    		glPointSize(4);
    		glEnable(GL_POINT_SMOOTH);
    		pc->draw();
    }
    cam.end();
    
    ofDisableDepthTest();
    // UI
    ofSetColor(0, 0, 0, 100);
    ofRect(20, 20, 200, 50);
    ofSetColor(255, 255, 255);
    if(running){
    	ofDrawBitmapString("Running", 35, 35);
    } else {
    	ofDrawBitmapString("Paused", 35, 35);
    }
    ofEnableDepthTest();
}

//--------------------------------------------------------------
void ofApp::updateCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
		if(!running) return;
		std::vector<ofVec3f> points;
		points.resize(cloud->points.size()); // allocate full point cloud
		for(unsigned int i = 0; i < cloud->points.size(); ++i){
			points[i] = ofVec3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		}
		ofMesh *newPC = new ofMesh();
		newPC->setMode(OF_PRIMITIVE_POINTS);
		newPC->addVertices(points);
		pointCloud.reset(newPC); // do the transfer now
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch(key)
    {
    case 'f':
        ofToggleFullscreen();
        break;
    case 'p':
    case ' ':
        running = !running;
        break;
    default:
        std::cout << "key '" << key << "' pressed.\n";
        break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{

}

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------

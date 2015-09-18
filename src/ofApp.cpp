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
    // ofEnableLighting();
    // ofEnableAlphaBlending();
    // ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofEnableSmoothing();
    cam.setDistance(1);

    // light the scene to show off why normals are important
    // light.enable();
    // light.setPointLight();
    // light.setPosition(0, 0, 10);

    std::cout << "Initializing Depth Sensor with OpenNI2.\n";
    pcGrabber = new pcl::io::OpenNI2Grabber();
    
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = 
    	boost::bind(&ofApp::updateCloud, this, _1);
    pcGrabber->registerCallback(f);
    pcGrabber->start();
    
    // setup ui
    std::cout << "Initializing UI.\n";
    // load the button images
    const char * files[] = {
      "capture.png", "running.png"
    };
    ui.resize(2);
    ui[Running].state = true; // set running by default
    int m = 20;
    int dx = 0;
    for(unsigned int i = 0; i < ui.size(); ++i){
        // load image
        ui[i].data.loadImage(files[i]);
        // set position given width
        dx += ui[i].data.getWidth() + m;
        ui[i].dx = dx;
        ui[i].dy = m;
    }
}

//--------------------------------------------------------------
void ofApp::update()
{
}

//--------------------------------------------------------------
void ofApp::draw()
{
    // ofEnableLighting();
    ofSetColor(255, 255, 255, 255);

    // draw background
    ofColor centerColor = ofColor(85, 78, 68);
    ofColor edgeColor(0, 0, 0);
    ofBackgroundGradient(centerColor, edgeColor, OF_GRADIENT_CIRCULAR);

    cam.begin();
    // display point cloud
    ofPushMatrix();
    // ofScale(1, 1, 1);
		ofRotate(-90, 1, 0, 0);
		ofTranslate(0, 0, -minZ * 0.5);
    for(int i = CAPTURE_MEMORY - 1; i >= 0; --i){
				ofMeshPtr pc = pointCloud[i]; // create a copy so that it may be replaced while being used (from OpenNI update)
				if(pc) {
						int I = 255 / (i + 1);
						int A = 150 / (i + 1);
						ofSetColor(I, I, I, A);
						glPointSize(float(CAPTURE_MEMORY) / float(i + 1));
						glEnable(GL_POINT_SMOOTH);
						pc->draw();
				}
    }
    ofPopMatrix();
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
    for(unsigned int i = 0; i < ui.size(); ++i){
				if(ui[i].state){
						ofSetColor(100, 100, 255, 255);
				} else if(ui[i].hover){
						ofSetColor(255, 255, 255, 255);
				} else  {
						ofSetColor(255, 255, 255, 70);
				}
				ui[i].draw();
    }
    ofEnableDepthTest();
}

//--------------------------------------------------------------
void ofApp::updateCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
		if(!pcGrabber->isRunning()) return;
		if(!ui[Capture].state) {
				pcGrabber->stop(); // note: we can still use the current point captured point cloud	
		}
		
		// save pc in capture
		if(ui[Capture].state){
				std::stringstream file("capture");
				file << captured << ".pcd";
				pcl::io::savePCDFile(file.str(), cloud, true); // binary so we can map directly to disk
				++captured; // increment capture index
		}
		
		// create new point cloud mesh
		std::vector<ofVec3f> points;
		points.resize(cloud->points.size()); // allocate full point cloud
		float minZ, maxZ;
		minZ = maxZ = cloud->points.size() ? cloud->points[0].z : 0;
		for(unsigned int i = 0; i < cloud->points.size(); ++i){
				points[i] = ofVec3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		}
		ofMesh *newPC = new ofMesh();
		newPC->setMode(OF_PRIMITIVE_POINTS);
		newPC->addVertices(points);
		this->minZ = 0.9 * this->minZ + 0.1 * minZ;
		this->maxZ = 0.9 * this->maxZ + 0.1 * maxZ;
		
		// transfer it
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
		for(unsigned int i = 0; i < ui.size(); ++i){
        ui[i].hover = ui[i].contains(x, y);
    }
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
		Action action = Unknown;
		for(unsigned int i = 0; i < ui.size(); ++i){
      	if(ui[i].hover){
          	ui[i].state = !ui[i].state;
          	action = static_cast<Action>(i);
      	}
    }
    // capture action
    bool newState = ui[action].state;
    switch(action){
    		case Capture:
    				// start a capture (end = do nothing special)
    				if(newState){
    						// start capture
    						capture = 0;
    				}
    				break;
    		case Running:
    				// only resume, we stop from the event thread
    				if(newState){
    						pcGrabber->start();
    				}
    				break;
    }
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

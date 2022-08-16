//
// Created by wj on 22. 6. 7.
//

//#include <OGRE/OgreManualObject.h>
//#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/uniform_string_stream.h>
#include <chrono>

#include "heightmap_rviz_plugin/HeightmapVisual.hpp"
#include <depalletizing_mapping_core/depalletizing_mapping_core.hpp>
#include <depalletizing_mapping_ros/depalletizing_mapping_ros.hpp>

namespace depalletizing_mapping_rviz_plugin
{
	HeightmapVisual::HeightmapVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode)
	{
		mSceneManager = sceneManager;
		mFrameNode = parentNode->createChildSceneNode();
//		mCylinderShapes.resize(msgSize);
//		for (int i = 0; i < msgSize; i++)
//		{
//			mCylinderShapes[i].reset(new rviz::Shape(rviz::Shape::Cylinder, mSceneManager, mFrameNode));
//		}
// 		-> move to SetMessgae Function
	}

	HeightmapVisual::~HeightmapVisual()
	{
		// Destroy the frame node.
		mSceneManager->destroySceneNode(mFrameNode);
	}

//	scale.x is diameter in x direction, scale.y in y direction, by setting these to different values you get an ellipse instead of a circle.
//	Use scale.z to specify the height.
	void HeightmapVisual::SetMessage(const heightmap_msgs::Heightmap::ConstPtr& msg)
	{
        ROS_INFO("SetMessage");
		// move from constructor for solve core dumped!
		int msgSize = msg->points.size();
		mCylinderShapes.resize(msgSize);
		for (int i = 0; i < msgSize; i++)
		{
			mCylinderShapes[i].reset(new rviz::Shape(rviz::Shape::Cylinder, mSceneManager, mFrameNode));
		}

		const float resolution = msg->resolution;

		std::vector<Ogre::Vector3> cylinderScales;
		cylinderScales.resize(msgSize);
		std::vector<Ogre::Vector3> cylinderPositions;
		cylinderPositions.resize(msgSize);
		std::vector<Ogre::Quaternion> cylinderOrientations;
		cylinderOrientations.resize(msgSize);

		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//		std::cout << msg->points.size() << ", " << mCylinderShapes.size() << std::endl;
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		for (int i = 0; i < msgSize; i++)
		{
			float x = msg->points[i].x;
			float y = msg->points[i].y;
			float z = msg->points[i].z;

			cylinderScales[i] = Ogre::Vector3(resolution, y, resolution);
			mCylinderShapes[i]->setScale(cylinderScales[i]);

			cylinderPositions[i] = Ogre::Vector3(x, y / 2, z);
			mCylinderShapes[i]->setPosition(cylinderPositions[i]);

			cylinderOrientations[i] = Ogre::Quaternion(1, 0, 0, 0);
			mCylinderShapes[i]->setOrientation(cylinderOrientations[i]);
		}
        ROS_INFO("SetMessage End");
	}

	void HeightmapVisual::SetFramePosition(const Ogre::Vector3& position)
	{
		mFrameNode->setPosition(position);
	}

	void HeightmapVisual::SetFrameOrientation(const Ogre::Quaternion& orientation)
	{
		mFrameNode->setOrientation(orientation);
	}

	void HeightmapVisual::SetColor(float r, float g, float b, float a)
	{
		for (int i = 0; i < mCylinderShapes.size(); i++)
		{
			mCylinderShapes[i]->setColor(r, g, b, a);
		}
	}

	void HeightmapVisual::SetColor(float a)
	{
		for (int i = 0; i < mCylinderShapes.size(); i++)
		{
			float cylinderHeight = -(mCylinderShapes[i]->getPosition().y * 2);
			if (cylinderHeight < 0.1f)
			{
				mCylinderShapes[i]->setColor(0, 0.458824, 1, a);	// blue
			}
			else if (cylinderHeight < 0.2f)
			{
				mCylinderShapes[i]->setColor(0.988235, 0.913725, 0.309804, a);	// yellow
			}
			else if (cylinderHeight < 0.3f)
			{
				mCylinderShapes[i]->setColor(0.541176, 0.886275, 0.203922, a);	// green
			}
			else
			{
				mCylinderShapes[i]->setColor(0.937255, 0.160784, 0.160784, a); // red
			}
		}
	}
}
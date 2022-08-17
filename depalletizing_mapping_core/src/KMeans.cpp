//
// Created by wj on 22. 7. 25.
//

#include "depalletizing_mapping_utils/KMeans.hpp"

namespace depalletizing_mapping
{
	KMeans::KMeans()
	{
	}

	KMeans::KMeans(std::vector<Point3D> data, float k)
			: mData(std::move(data))
	{
	}

	std::vector<Point3D> KMeans::GetData() const
	{
		return mData;
	}

	void KMeans::SetData(const std::vector<Point3D>& data)
	{
		mData = data;
	}
    std::vector<Point3D> KMeans::GetOutputData() const
    {
        return mOutputData;
    }
    void KMeans::SetOutputData(const std::vector<Point3D>& outputData)
    {
        mOutputData = outputData;
    }

	Point2D KMeans::GetOneCentroid() const
	{
		return mCentroids[0];
	}

	void KMeans::FromPCD(const std::string& inputPath)
	{
		mData.reserve(307200);

		std::ifstream fin;
		fin.open(inputPath);
		std::string line;

		if (fin.is_open())
		{
			int num = 1;
			while (!fin.eof())
			{
				getline(fin, line);
				if (num > 10)
				{
					if (fin.eof())	// new !!!
					{
						break;
					}				// new !!!
					float x, y, z;
					std::istringstream iss(line);
					iss >> x >> y >> z;

					Point3D pointXYZ = {x, y, z};
					mData.push_back(pointXYZ);
				}
				num++;
			}
		}
		fin.close();

		std::cout << "FromPCD : " << mData.size() << std::endl;
	}

	void KMeans::ToPCD(const std::vector<Point3D>& data, int num)
	{
		std::string outputPath = "/home/wj/Desktop/Data/kmeans/output_data/";
		time_t t;
		struct tm* timeinfo;
		time(&t);
		timeinfo = localtime(&t);

		std::string hour, min;

		if (timeinfo->tm_hour < 10) hour = "0" + std::to_string(timeinfo->tm_hour);
		else hour = std::to_string(timeinfo->tm_hour);

		if (timeinfo->tm_min < 10) min = "0" + std::to_string(timeinfo->tm_min);
		else min = std::to_string(timeinfo->tm_min);

		std::string filePath = outputPath + hour + min + "_" + std::to_string(num) + ".pcd";

		std::ofstream fout;
		fout.open(filePath);

		fout << "VERSION" << std::endl;
		fout << "FIELDS x y z" << std::endl;
		fout << "SIZE 4 4 4" << std::endl;
		fout << "TYPE F F F" << std::endl;
		fout << "COUNT 1 1 1" << std::endl;
		fout << "WIDTH 1" << std::endl;
		fout << "HEIGHT " << data.size() << std::endl;
		fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
		fout << "POINTS " << data.size() << std::endl;
		fout << "DATA ascii" << std::endl;

		for (int i = 0; i < data.size(); i++)
		{
			fout << data[i].GetX() << " " << data[i].GetY() << " " << data[i].GetZ() << "\n";
		}

		fout.close();
	}

	void KMeans::Run()
	{
		setInitialCentroid();

		bool iterate = false;
		int iterateNum = 0;
		int nearNumParameter = 20;
		while (!iterate)
		{
			assignCentroid();
			while (bUpdateCentroid())
			{
				std::cout << "iterate" << std::endl;
				iterateNum++;
				assignCentroid();
			}

			if (!bFindNearRate(nearNumParameter))
			{
				generateCentroid();
				nearNumParameter--;
			}
			else
			{
				iterate = true;
			}
		}
		std::cout << "iterate result : " << iterateNum << std::endl;
		clustering();
	}

	void KMeans::SaveResult()
	{
		for (int i = 0; i < mModels.size(); i++)
		{
			ToPCD(mModels[i].GetData(), i);
		}
		std::cout << "save model num : " << mModels.size() << std::endl;
	}

    void KMeans::SaveOneResult()
    {
        SetOutputData(mModels[0].GetData());
    }

	void KMeans::setInitialCentroid()
	{
		std::sort(mData.begin(), mData.end(), Point3D::DescendingByX);
		float maxX = mData[0].GetX();
		float minX = mData[mData.size() - 1].GetX();

		std::sort(mData.begin(), mData.end(), Point3D::DescendingByZ);
		float maxZ = mData[0].GetZ();
		float minZ = mData[mData.size() - 1].GetZ();

		std::random_device randomDevice;
		std::mt19937 generator(randomDevice());
		std::uniform_real_distribution<float> randomX(minX, maxX);
		std::uniform_real_distribution<float> randomZ(minZ, maxZ);

		Point2D centroid = { randomX(generator), randomZ(generator) };
		std::cout << "setInitialCentroid -> random centroid : " << centroid.GetX() << ", " << centroid.GetZ() << std::endl;
		mCentroids.push_back(centroid);
	}

	void KMeans::assignCentroid()
	{
		for (int i = 0; i < mData.size(); i++)
		{
			std::vector<float> distanceBetweenCentroid;
			for (int j = 0; j < mCentroids.size(); j++)
			{
				distanceBetweenCentroid.push_back(mData[i].DistanceBetween2D(mCentroids[j]));
			}

//			if (distanceBetweenCentroid.size() == 1)
//			{
//				mData[i].SetCentroid(mCentroids[0]);
//			}
//			else
//			{
			int minDistanceCentroidIndex = std::min_element(distanceBetweenCentroid.begin(), distanceBetweenCentroid.end()) - distanceBetweenCentroid.begin();
			mData[i].SetCentroid(mCentroids[minDistanceCentroidIndex]);
//			}
		}
	}

	// after update : true, Not need : false
	bool KMeans::bUpdateCentroid()
	{
		std::vector<bool> checkUpdates;
		for (int cenIndex = 0; cenIndex < mCentroids.size(); cenIndex++)
		{
			bool checkUpdate = true;
			float newCentroidX = 0;
			float newCentroidZ = 0;
			float num = 0;

			for (int DataIndex = 0; DataIndex < mData.size(); DataIndex++)
			{
				if (mCentroids[cenIndex].bIsEqual(mData[DataIndex].GetCentroid()))
				{
					newCentroidX += mData[DataIndex].GetX();
					newCentroidZ += mData[DataIndex].GetZ();
					num++;
				}
			}

			if (num != 0)
			{
				float updateX = newCentroidX / num;
				float updateZ = newCentroidZ / num;

				if (mCentroids[cenIndex].GetX() == updateX && mCentroids[cenIndex].GetZ() == updateZ)
				{
					checkUpdate = false;
					checkUpdates.push_back(checkUpdate);
					continue;
				}

				mCentroids[cenIndex].SetX(updateX);
				mCentroids[cenIndex].SetZ(updateZ);

				std::cout << "bUpdateCentroid -> mCentroids : " << mCentroids[cenIndex].GetX() << ", " << mCentroids[cenIndex].GetZ() << std::endl;

				checkUpdates.push_back(checkUpdate);
			}
		}

		bool returnCheck = false;
		for (int i = 0; i < checkUpdates.size(); i++)
		{
			returnCheck = returnCheck || checkUpdates[i];
		}
		return returnCheck;
	}

	// Good near : true, Not : false
	bool KMeans::bFindNearRate(int nearNumParameter)
	{
		if (nearNumParameter == 5)
		{
			return true;
		}

		float resolution = 0.015625f;
		float diameter = 2 * resolution * std::sqrt(2.0f);
//		int nearParameter = 20;

		for (int cenIndex = 0; cenIndex < mCentroids.size(); cenIndex++)
		{
			float dataNum = 0;
			float nearNum = 0;

			for (int i = 0; i < mData.size(); i++)
			{
				if (mCentroids[cenIndex].bIsEqual(mData[i].GetCentroid()))
				{
					dataNum++;
					if (mData[i].DistanceBetween2D(mCentroids[cenIndex]) < diameter)
					{
						nearNum++;
					}
				}
			}
			if (dataNum !=0)
			{
				std::cout << "bFindNearRate : dataNum : " << dataNum << ", nearNum : " << nearNum << std::endl;
				if (nearNum < nearNumParameter)
				{
					std::cout << "bFindNearRate : Need to separate Centroid" << std::endl;
					return false;
				}
			}
		}
		std::cout << "bFindNearRate : Good Centroid" << std::endl;
		return true;
	}

	void KMeans::generateCentroid()
	{
		// 기존의 마지막 centroid 기준으로 가장 멀리있는 data를 새로운 centroid로.
		float distance = 0;
		int index = 0;

		for (int i = 0; i < mData.size(); i++)
		{
			float measured = mData[i].DistanceBetween2D(mCentroids[mCentroids.size() - 1]);
			if (distance < measured)
			{
				distance = measured;
				index = i;
			}
		}

		Point2D newCentroid = { mData[index].GetX(), mData[index].GetZ() };
		mCentroids.push_back(newCentroid);
	}

	void KMeans::clustering()
	{
		for (int i = 0; i < mCentroids.size(); i++)
		{
			std::vector<Point3D> data;

			for (int j = 0; j < mData.size(); j++)
			{
				if (mCentroids[i].bIsEqual(mData[j].GetCentroid()))
				{
					data.push_back(mData[j]);
				}
			}

			PlaneModel model(data);
			mModels.push_back(model);
		}
	}

}
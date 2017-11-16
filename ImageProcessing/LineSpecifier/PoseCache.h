/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2016-02-05 #$
 **********************************************************************/

#ifndef POSECACHE_H_
#define POSECACHE_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

#include <iostream>
#include "CVMath.h"
#include "Pose3D.h"

#include "ImageProcessingUtils.h"

#define POSE_CACHE_LOG_CONSOLE 0

struct CarPose {
	cv::Point2f position;
	tFloat32 yaw;
	tTimeStamp adtfTime;
	tUInt32 arduinoTime;

	CarPose () {
		this->yaw = 0;
		this->adtfTime = 0;
		this->arduinoTime = 0;
	}

	CarPose (cv::Point2f pose, tFloat32 yaw, tTimeStamp adtfTime, tUInt32 arduinoTime) {
		this->position = pose;
		this->yaw = yaw;
		this->adtfTime = adtfTime;
		this->arduinoTime = arduinoTime;
	}
};

class PoseCache {
	std::deque<CarPose> _queue;

	tInt64 _adtfArduinoTimeDiff;

	/**
	 * Synchronization
	 */
	cCriticalSection _criticalSection;
public:

	bool IsEmpty() {
		__synchronized_obj(_criticalSection);
		return _queue.empty();
	}

	void PushFront(CarPose p) {
		__synchronized_obj(_criticalSection);

		size_t size = _queue.size();
		if(size == 0) {
			_adtfArduinoTimeDiff = p.adtfTime - p.arduinoTime;
			_queue.push_front(p);
		} else {
			CarPose last = _queue.front();
			if(last.arduinoTime < p.arduinoTime) {
				_adtfArduinoTimeDiff = 0.3*(p.adtfTime - p.arduinoTime) + 0.7 * _adtfArduinoTimeDiff;
				_queue.push_front(p);
			} else if(last.arduinoTime > p.arduinoTime) {
				tUInt32 diff = last.arduinoTime - p.arduinoTime;

				if(diff > 4294917296) { //256*256*256*256 - 50000 [microseconds]
					//time overflow
					_adtfArduinoTimeDiff = 0.3*(p.adtfTime - p.arduinoTime) + 0.7 * (_adtfArduinoTimeDiff + 4294967296);
					_queue.push_front(p);
				} else {
					//reset
					_adtfArduinoTimeDiff = p.adtfTime - p.arduinoTime;
					_queue.push_front(p);
				}
			} else {
				//TODO: rework adtfArduinoTimeDiff
				_queue.push_front(p);
			}
		}

		if(size > 45) {
			_queue.resize(40);
		}
	}

	CarPose GetCurrent() {
		__synchronized_obj(_criticalSection);
		return _queue.front();
	}

	CarPose GetLast() {
		__synchronized_obj(_criticalSection);
		if(_queue.size() > 1) {
			return _queue.at(1);
		}
		return CarPose();
	}

	tFloat32 GetLastDistance() {
		__synchronized_obj(_criticalSection);
		if(_queue.size() > 1) {
			return cv::norm(_queue.at(0).position - _queue.at(1).position);
		} else {
			return -1;
		}
	}

	CarPose GetGlobalPose(cv::Point2f position, tFloat32 yaw) {
		//TODO: not tested
		__synchronized_obj(_criticalSection);

		if(_queue.size() > 0) {
			CarPose car = _queue.front();
			CarPose point;
			point.position = CVMath::RotateCW(position, -car.yaw) + car.position;
			point.yaw = car.yaw + yaw;
			return point;
		} else {
			return CarPose();
		}
	}

	CarPose GetLocalPose(CarPose pos) {
		//TODO: not tested
		__synchronized_obj(_criticalSection);

		if(_queue.size() > 0) {
			CarPose car = _queue.front();
			CarPose point;
			point.position = CVMath::RotateCW(pos.position - car.position, car.yaw);
			point.yaw = pos.yaw - car.yaw;
			return point;
		} else {
			return CarPose();
		}
	}

	tFloat32 GetSpeed() {
		__synchronized_obj(_criticalSection);

		if ( _queue.size() < 2) {
			return 0;
		}

		cv::Point2f posDiff = (_queue[0].position - _queue[1].position);
		tTimeStamp timeDiff = _queue[0].arduinoTime - _queue[1].arduinoTime;

		if(timeDiff < 0) {
			timeDiff = - timeDiff;
		}

		if(timeDiff == 0) {
			return 0;
		}

		return cv::norm(posDiff) * 1000000 / timeDiff;
	}

	tFloat32 GetYawDiff() {
		__synchronized_obj(_criticalSection);

		if ( _queue.size() < 2) {
			return 0;
		}

		tFloat32 yaw0 = _queue[0].yaw;
		if(yaw0 > CV_PI) {
			yaw0 -= 2*CV_PI;
		} else if(yaw0 < -CV_PI) {
			yaw0 += 2*CV_PI;
		}

		tFloat32 yaw1 = _queue[1].yaw;
		if(yaw1 > CV_PI) {
			yaw1 -= 2*CV_PI;
		} else if(yaw1 < -CV_PI) {
			yaw1 += 2*CV_PI;
		}

		tFloat32 yawDiff =  _queue[0].yaw - _queue[1].yaw;
		if(yawDiff > CV_PI) {
			yawDiff -= 2*CV_PI;
		} else if(yawDiff < -CV_PI) {
			yawDiff += 2*CV_PI;
		}

		return yawDiff;
	}

	/**
	 *
	 * @param poses
	 * @param frameCoord
	 * @return local position
	 */
	cv::Point2f InterpolatePoses(deque<Pose3D> poses, cv::Point2f frameCoord) {
		if (poses.size() < 2 || _queue.size() < 2) {
			return cv::Point2f(0,0);
		}

		int countPoints = 0;
		cv::Point2f mean;

#if POSE_CACHE_LOG_CONSOLE
		for(size_t i = 0; i < _queue.size(); i++) {
			std::cout << "car pose " << i << " " << _queue[i].position.x << " " << _queue[i].position.y << "yaw " << _queue[i].yaw  << " " << _queue[i].adtfTime << std::endl;
		}
#endif

		for(size_t i = 0; i < poses.size(); i++) {
			if(poses[i].valid) {
				cv::Point2f stopPosition = ImageUtils::ConvertToWorldCoordinates(poses[i].pos, frameCoord);

#if POSE_CACHE_LOG_CONSOLE
				std::cout << "LS: local pose " << i << " " << stopPosition.x << " " <<stopPosition.y  << " " << poses[i].time << std::endl;
#endif

				__synchronized_obj(_criticalSection);

				size_t k = 0;
				tTimeStamp time = poses[i].time;

				while (k + 1 < _queue.size()) {
					CarPose last = _queue.at(k);
					CarPose preLast = _queue.at(k+1);

					if(time > last.adtfTime) {
						break;
					}

					if(preLast.adtfTime <= time && time <= last.adtfTime) {
						tTimeStamp timeDiffDivisor = last.adtfTime - preLast.adtfTime;
						cv::Point2f positionDiff = last.position - preLast.position;

						float yawDiff = last.yaw - preLast.yaw;
						if(yawDiff > CV_PI) {
							yawDiff -= 2*CV_PI;
						} else if(yawDiff < -CV_PI) {
							yawDiff += 2*CV_PI;
						}

						tTimeStamp timeDiff = time - preLast.adtfTime;

						CarPose inter = CarPose(preLast.position + (float(timeDiff) / timeDiffDivisor) * positionDiff, preLast.yaw + (yawDiff * timeDiff) / timeDiffDivisor, time, true);

						cv::Point2f pos = CVMath::RotateCW(stopPosition, -inter.yaw) + inter.position;
						mean += pos;
						countPoints++;

#if POSE_CACHE_LOG_CONSOLE
						std::cout << "LS: car pose " << i << " " << inter.position.x << " " << inter.position.y  << " yaw " <<inter.yaw << std::endl;
						std::cout << "LS: global pose " << i << " " << pos.x << " " <<pos.y << std::endl;
#endif

						break;
					}

					k++;

				}

				k-= 2;
				if(k < 0) {
					k = 0;
				}
			}
		}

		if (countPoints > 0) {
			mean = mean / countPoints;

			__synchronized_obj(_criticalSection);

			return CVMath::RotateCW(mean - _queue.front().position, _queue.front().yaw);
		} else {
			std::cout << "LS countPoints == 0" << std::endl;
			return cv::Point2f(0,0);
		}
	}

	PoseCache() {
		_adtfArduinoTimeDiff = 0;
	}

	virtual ~PoseCache() {

	}
};

#endif /* POSECACHE_H_ */

/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <mutex>
#include "Eigen/Core"
#include <deque>

#ifndef SUNRISE_X3
    #include <pangolin/pangolin.h>
#endif

namespace ORB_SLAM3
{
    struct ViewData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double t;
        Eigen::Vector3d pos;
        Eigen::Vector3d atti;

        bool bRtkslip;

        ViewData() : bRtkslip(false) {}
    };

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Viewer();

#ifndef SUNRISE_X3

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void AddRtkViewdata(const ViewData &rtkdata);
        void AddEkfViewdata(const ViewData &ekfdata);

        void ReplaceRtkEkfViewdatas(const std::vector<ViewData> &veRtkdata, const std::vector<ViewData> &veEkfdata);

        void Stop();

    private:

        void PrepareData(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, 
            std::deque<ViewData> &rtkdatas, std::deque<ViewData> &ekfdatas);
#endif

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
        float mLineWidth;
        std::mutex mMutex;
        std::deque<ViewData> mRtkdatas;
        std::deque<ViewData> mEkfdatas;
        bool mRunning;
        
    };

}

#endif // VIEWER_H

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

#include "Viewer.h"
#include "ImuTypes.h"
#include "Common.h"
// #include "Thirdparty/Sophus/sophus/se3.hpp"
#include "sophus/se3.hpp"

namespace ORB_SLAM3
{
    Viewer::Viewer()
    {
        mViewpointX = 0.0;
        mViewpointY = -0.7;
        mViewpointZ = -3.5;
        mViewpointF = 500.0;

        mLineWidth = 2.0;
    }

#ifndef SUNRISE_X3
    void Viewer::Run()
    {
        pangolin::CreateWindowAndBind("Gps-Dr : Map Viewer", 1024, 768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
        pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
        pangolin::Var<bool> menuTopView("menu.Top View", false, false);
        pangolin::Var<bool> menuShowRtk("menu.Show Gps", true, true);
        pangolin::Var<bool> menuShowEkf("menu.Show Dr", true, true);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc, Twr;
        Twc.SetIdentity();
        pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
        Ow.SetIdentity();

        Eigen::Matrix4f Twc_tmp;

        bool bFollow = true;
        bool bCameraView = true;

        mRunning = true;

        while (1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            std::deque<ViewData> rtkdatas;
            std::deque<ViewData> ekfdatas;
            PrepareData(Twc, Ow, rtkdatas, ekfdatas);

            if (menuFollowCamera && bFollow)
            {
                if (bCameraView)
                    s_cam.Follow(Twc);
                else
                    s_cam.Follow(Ow);
            }
            else if (menuFollowCamera && !bFollow)
            {
                if (bCameraView)
                {
                    s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                    s_cam.Follow(Twc);
                }
                else
                {
                    s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
                    s_cam.Follow(Ow);
                }
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if (menuCamView)
            {
                menuCamView = false;
                bCameraView = true;
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
            }

            if (menuTopView)
            {
                menuTopView = false;
                bCameraView = false;
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
                s_cam.Follow(Ow);
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // 绘图代码
            // 绘制rtk轨迹
            int size = rtkdatas.size();
            if(menuShowRtk)
            {
                // 线绘制rtk正常轨迹
                glLineWidth(mLineWidth);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
                for(int i=0; i<size; i++)
                {
                    if(i>0 && (!rtkdatas.at(i-1).bRtkslip && !rtkdatas.at(i).bRtkslip))
                    {
                        Eigen::Vector3d &pos1 = rtkdatas.at(i-1).pos;
                        Eigen::Vector3d &pos2 = rtkdatas.at(i).pos;
                        glVertex3f(pos1(0), pos1(1), 0);
                        glVertex3f(pos2(0), pos2(1), 0);
                    }
                }
                glEnd();

                // 再绘制rtk跳变轨迹
                glLineWidth(mLineWidth);
                glColor3f(0.2f, 0.0f, 0.2f);
                glBegin(GL_LINES);
                for(int i=0; i<size; i++)
                {
                    if(i>0 && (rtkdatas.at(i-1).bRtkslip || rtkdatas.at(i).bRtkslip))
                    {
                        Eigen::Vector3d &pos1 = rtkdatas.at(i-1).pos;
                        Eigen::Vector3d &pos2 = rtkdatas.at(i).pos;
                        glVertex3f(pos1(0), pos1(1), 0);
                        glVertex3f(pos2(0), pos2(1), 0);
                    }
                }
                glEnd();

            }

            // 绘制ekf轨迹
            if(menuShowEkf)
            {
                size = ekfdatas.size();
                glLineWidth(mLineWidth);
                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
                for(int i=0; i<size; i++)
                {
                    if(i>0)
                    {
                        Eigen::Vector3d &pos1 = ekfdatas.at(i-1).pos;
                        Eigen::Vector3d &pos2 = ekfdatas.at(i).pos;
                        glVertex3f(pos1(0), pos1(1), 0);
                        glVertex3f(pos2(0), pos2(1), 0);
                    }
                }
                glEnd();

                // 绘制当前位置点和姿态
                if(ekfdatas.size() > 0)
                {
                    glPushMatrix();

                    Eigen::Matrix4f Twc_tmp;
                    for(int i=0; i<4; i++)
                    {
                        for(int j=0; j<4; j++)
                        {
                            Twc_tmp(i,j) = Twc.m[4 * j + i];
                        }
                    }

                    glMultMatrixf((GLfloat *)Twc_tmp.data());

                    glLineWidth(mLineWidth * 3);

                    // 1
                    glColor3f(0.0f, 1.0f, 0.0f);
                    glBegin(GL_LINES);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0.1, 0, 0);
                    glEnd();
                    // 2
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0, 0.1, 0);
                    glEnd();
                    // 3
                    glColor3f(0.0f, 0.0f, 1.0f);
                    glBegin(GL_LINES);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0, 0, 0.1);
                    glEnd();

                    glPopMatrix();
                    glEnd();
                }
            }

            pangolin::FinishFrame();

            if(!mRunning)
            {
                break;
            }

        }

    }

    void Viewer::PrepareData(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, 
        std::deque<ViewData> &rtkdatas, std::deque<ViewData> &ekfdatas)
    {
        {
            std::lock_guard<std::mutex> lk(mMutex);

            Eigen::Matrix4f Twc;          
            if(mEkfdatas.size() > 0)
            {
                ViewData &viewData = mEkfdatas.back();

                Sophus::SE3f pose(Attitude2R(viewData.atti).cast<float>(), viewData.pos.cast<float>());
                Twc = pose.matrix();

                Twc.setIdentity();
                Twc.block<3,3>(0,0) = Attitude2R(viewData.atti).cast<float>();
                Twc.block<3,1>(0,3) = viewData.pos.cast<float>();
            }
            else if(mRtkdatas.size() > 0)
            {
                ViewData &viewData = mRtkdatas.back();
                Sophus::SE3f pose(Eigen::Matrix3f::Identity(), viewData.pos.cast<float>());
                Twc = pose.matrix();
            }

            for (int i = 0; i < 4; i++)
            {
                M.m[4 * i] = Twc(0, i);
                M.m[4 * i + 1] = Twc(1, i);
                M.m[4 * i + 2] = Twc(2, i);
                M.m[4 * i + 3] = Twc(3, i);
            }

            MOw.SetIdentity();
            MOw.m[12] = Twc(0, 3);
            MOw.m[13] = Twc(1, 3);
            MOw.m[14] = Twc(2, 3);

            rtkdatas = mRtkdatas;
            ekfdatas = mEkfdatas;
        }
    }

    void Viewer::AddRtkViewdata(const ViewData &rtkdata)
    {
       {
            std::lock_guard<std::mutex> lk(mMutex);
            mRtkdatas.push_back(rtkdata);
       }
    }

    void Viewer::AddEkfViewdata(const ViewData &ekfdata)
    {
        {
            std::lock_guard<std::mutex> lk(mMutex);
            mEkfdatas.push_back(ekfdata);
        }
    }

    void Viewer::ReplaceRtkEkfViewdatas(const std::vector<ViewData> &veRtkdata, const std::vector<ViewData> &veEkfdata)
    {
        std::lock_guard<std::mutex> lk(mMutex);
        mRtkdatas.clear();
        mEkfdatas.clear();
        mRtkdatas.insert(mRtkdatas.end(), veRtkdata.begin(), veRtkdata.end());
        mEkfdatas.insert(mEkfdatas.end(), veEkfdata.begin(), veEkfdata.end());
    }


    void Viewer::Stop()
    {
        mRunning = false;
    }
#endif

}

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \Author Ioan Sucan */

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>


TEST(SpherePointContainment, SimpleInside)
{
    shapes::Sphere shape(1.0);
    bodies::Body* sphere = new bodies::Sphere(&shape);
    sphere->setScale(1.05);
    bool contains = sphere->containsPoint(0,0,1.0);
    random_numbers::RandomNumberGenerator r;
    Eigen::Vector3d p;
    EXPECT_TRUE(sphere->samplePointInside(r, 100, p));
    EXPECT_TRUE(sphere->containsPoint(p));
    EXPECT_TRUE(contains);
    delete sphere;
}

TEST(SpherePointContainment, SimpleOutside)
{
    shapes::Sphere shape(1.0);
    bodies::Body* sphere = new bodies::Sphere(&shape);
    sphere->setScale(0.95);
    bool contains = sphere->containsPoint(0,0,1.0);
    delete sphere;
    EXPECT_FALSE(contains);
}

TEST(SpherePointContainment, ComplexInside)
{
    shapes::Sphere shape(1.0);
    bodies::Body* sphere = new bodies::Sphere(&shape);
    sphere->setScale(0.95);
    Eigen::Affine3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(1.0,1.0,1.0);
    sphere->setPose(pose);
    bool contains = sphere->containsPoint(0.5,1,1.0);
    delete sphere;
    EXPECT_TRUE(contains);
}

TEST(SpherePointContainment, ComplexOutside)
{
    shapes::Sphere shape(1.0);
    bodies::Body* sphere = new bodies::Sphere(&shape);
    sphere->setScale(0.95);
    Eigen::Affine3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(1.0,1.0,1.0);
    sphere->setPose(pose);
    bool contains = sphere->containsPoint(0.5,0.0,0.0);
    delete sphere;
    EXPECT_FALSE(contains);
}

TEST(SphereRayIntersection, SimpleRay1)
{
    shapes::Sphere shape(1.0);
    bodies::Body* sphere = new bodies::Sphere(&shape);
    sphere->setScale(1.05);

    Eigen::Vector3d ray_o(5, 0, 0);
    Eigen::Vector3d ray_d(-1, 0, 0);
    EigenSTL::vector_Vector3d p;
    bool intersect = sphere->intersectsRay(ray_o, ray_d, &p);

    delete sphere;
    EXPECT_TRUE(intersect);
    EXPECT_EQ(2, (int)p.size());
    EXPECT_NEAR(p[0].x(), 1.05, 1e-6);
    EXPECT_NEAR(p[1].x(), -1.05, 1e-6);
}

TEST(SphereRayIntersection, SimpleRay2)
{
    shapes::Sphere shape(1.0);
    bodies::Body* sphere = new bodies::Sphere(&shape);
    sphere->setScale(1.05);

    Eigen::Vector3d ray_o(5, 0, 0);
    Eigen::Vector3d ray_d(1, 0, 0);
    EigenSTL::vector_Vector3d p;
    bool intersect = sphere->intersectsRay(ray_o, ray_d, &p);

    delete sphere;
    EXPECT_FALSE(intersect);
    EXPECT_EQ(0, (int)p.size());
}

TEST(BoxPointContainment, SimpleInside)
{
    shapes::Box shape(1.0, 2.0, 3.0);
    bodies::Body* box = new bodies::Box(&shape);
    box->setScale(0.95);
    bool contains = box->containsPoint(0,0,1.0);
    EXPECT_TRUE(contains);

    random_numbers::RandomNumberGenerator r;
    Eigen::Vector3d p;
    EXPECT_TRUE(box->samplePointInside(r, 100, p));
    EXPECT_TRUE(box->containsPoint(p));

    delete box;
}


TEST(BoxPointContainment, SimpleOutside)
{
    shapes::Box shape(1.0, 2.0, 3.0);
    bodies::Body* box = new bodies::Box(&shape);
    box->setScale(0.95);
    bool contains = box->containsPoint(0,0,3.0);
    delete box;
    EXPECT_FALSE(contains);
}


TEST(BoxPointContainment, ComplexInside)
{
    shapes::Box shape(1.0, 1.0, 1.0);
    bodies::Body* box = new bodies::Box(&shape);
    box->setScale(1.01);
    Eigen::Affine3d pose(Eigen::AngleAxisd(M_PI/3.0, Eigen::Vector3d::UnitX()));
    pose.translation() = Eigen::Vector3d(1.0,1.0,1.0); 
    box->setPose(pose);

    bool contains = box->containsPoint(1.5,1.0,1.5);
    EXPECT_TRUE(contains);

    random_numbers::RandomNumberGenerator r;
    Eigen::Vector3d p;
    for (int i = 0 ; i < 100 ; ++i)
    {
        EXPECT_TRUE(box->samplePointInside(r, 100, p));
	EXPECT_TRUE(box->containsPoint(p));
    }

    delete box;
}

TEST(BoxPointContainment, ComplexOutside)
{
    shapes::Box shape(1.0, 1.0, 1.0);
    bodies::Body* box = new bodies::Box(&shape);
    box->setScale(1.01);
    Eigen::Affine3d pose(Eigen::AngleAxisd(M_PI/3.0, Eigen::Vector3d::UnitX()));
    pose.translation() = Eigen::Vector3d(1.0,1.0,1.0); 
    box->setPose(pose);

    bool contains = box->containsPoint(1.5,1.5,1.5);
    delete box;
    EXPECT_FALSE(contains);
}

TEST(BoxRayIntersection, SimpleRay1)
{
    shapes::Box shape(1.0, 1.0, 3.0);
    bodies::Body* box = new bodies::Box(&shape);
    box->setScale(0.95);

    Eigen::Vector3d ray_o(10, 0.449, 0);
    Eigen::Vector3d ray_d(-1, 0, 0);
    EigenSTL::vector_Vector3d p;

    bool intersect = box->intersectsRay(ray_o, ray_d, &p);

    //    for (unsigned int i = 0; i < p.size() ; ++i)
    //        printf("intersection at %f, %f, %f\n", p[i].x(), p[i].y(), p[i].z());

    delete box;
    EXPECT_TRUE(intersect);
}


TEST(CylinderPointContainment, SimpleInside)
{
    shapes::Cylinder shape(1.0, 4.0);
    bodies::Body* cylinder = new bodies::Cylinder(&shape);
    cylinder->setScale(1.05);
    bool contains = cylinder->containsPoint(0, 0, 2.0);
    delete cylinder;
    EXPECT_TRUE(contains);
}


TEST(CylinderPointContainment, SimpleOutside)
{
    shapes::Cylinder shape(1.0, 4.0);
    bodies::Body* cylinder = new bodies::Cylinder(&shape);
    cylinder->setScale(0.95);
    bool contains = cylinder->containsPoint(0,0,2.0);
    delete cylinder;
    EXPECT_FALSE(contains);
}

TEST(CylinderPointContainment, CylinderPadding)
{
    shapes::Cylinder shape(1.0, 4.0);
    bodies::Body* cylinder = new bodies::Cylinder(&shape);
    bool contains = cylinder->containsPoint(0,1.01,0);
    EXPECT_FALSE(contains);
    cylinder->setPadding(.02);
    contains = cylinder->containsPoint(0,1.01,0);
    EXPECT_TRUE(contains);
    cylinder->setPadding(0.0);
    bodies::BoundingSphere bsphere;
    cylinder->computeBoundingSphere(bsphere);
    EXPECT_TRUE(bsphere.radius > 2.0);

    random_numbers::RandomNumberGenerator r;
    Eigen::Vector3d p;
    for (int i = 0 ; i < 1000 ; ++i)
    {
        EXPECT_TRUE(cylinder->samplePointInside(r, 100, p));
        EXPECT_TRUE(cylinder->containsPoint(p));
    }
    delete cylinder;
}

TEST(MeshPointContainment, Pr2Forearm)
{
    shapes::Mesh *ms = shapes::createMeshFromResource("file://" + (boost::filesystem::current_path() / "test/resources/forearm_roll.stl").string());
    EXPECT_EQ(ms->vertex_count, 2338);
    bodies::Body *m = new bodies::ConvexMesh(ms);
    Eigen::Affine3d t(Eigen::Affine3d::Identity());
    t.translation().x() = 1.0;
    EXPECT_FALSE(m->cloneAt(t)->containsPoint(-1.0, 0.0, 0.0));

    random_numbers::RandomNumberGenerator r;
    Eigen::Vector3d p;
    bool found = true;
    for (int i = 0 ; i < 10 ; ++i) {
      if (m->samplePointInside(r, 10000, p))
      {
        found = true;
        EXPECT_TRUE(m->containsPoint(p));
      }
    }
    EXPECT_TRUE(found);

    delete m;
    delete ms;
}

TEST(MergeBoundingSpheres, MergeTwoSpheres)
{
  std::vector<bodies::BoundingSphere> spheres;

  bodies::BoundingSphere s1;
  s1.center = Eigen::Vector3d(5.0, 0.0, 0.0);
  s1.radius = 1.0;

  bodies::BoundingSphere s2;
  s2.center = Eigen::Vector3d(-5.1, 0.0, 0.0);
  s2.radius = 1.0;

  spheres.push_back(s1);
  spheres.push_back(s2);

  bodies::BoundingSphere merged_sphere;
  bodies::mergeBoundingSpheres(spheres, merged_sphere);

  EXPECT_NEAR(merged_sphere.center[0], -.05, .00001);
  EXPECT_EQ(merged_sphere.radius, 6.05);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

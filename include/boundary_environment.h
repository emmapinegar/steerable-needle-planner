// BSD 3-Clause License

// Copyright (c) 2021, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#pragma once
#ifndef SNP_BOUND_ENVIRONMENG_H
#define SNP_BOUND_ENVIRONMENG_H

#include <nigh/lp_space.hpp>
#include <nigh/kdtree_batch.hpp>

#include "global_common.h"

using Vec3 = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
using SizeType = std::size_t;

struct BoundPoint {
    Vec3 point;
    Vec3 direction;
    Vec3 b;
    Vec3 m;
    BoundPoint(const Vec3& p, const Vec3& n, const Vec3& m_ = Vec3(0,0,0), const Vec3& b_ = Vec3(0,0,0)) : point(p), direction(n), m(m_), b(b_) {

    }
};

struct BoundNode {
    SizeType idx;
    BoundPoint bound;
    bool valid;
    BoundNode(const SizeType i, const Vec3& p, const Vec3& n) : idx(i), bound(p,n) {
        valid = true;
    }
};

struct BoundKey {
    const BoundPoint& operator() (const BoundNode& node) const {
        return node.bound;
    }
};


namespace unc::robotics::nigh::metric {
    template<>
    struct Space<BoundPoint, LP<4>> {
        using Type = BoundPoint;
        using Metric = LP<4>;
        using Distance = double;
        static constexpr int kDimensions = 3;

        static constexpr double R_INF = std::numeric_limits<double>::infinity();

        static bool isValid(const Type& bound) {
            return std::isfinite(bound.point[0]) && std::isfinite(bound.point[1]) && std::isfinite(bound.point[2]);
        }

        static double coeff(const BoundPoint& bound, std::size_t i) {
            return *(&bound.point[0] + i);
        }

        constexpr unsigned dimensions() const {
            return kDimensions;
        }

        static Distance distance(const Type& bound, const Type& query) {
            Vec3 r = bound.point - query.point;
            Distance dist = R_INF;
            if (query.direction.norm() > 0.5) {
                Vec3 r_hat = r.normalized();
                dist = r_hat.cross(query.direction).norm();
                
                // if (dist < 10) {
                //     double rdot = 1/std::abs(r_hat.dot(query.direction)) - 1; 
                //     double rcross = r_hat.cross(query.direction).norm();
                //     // std::cout << "bound: " << bound.point.transpose() << " query: " << query.point.transpose() << " r: " << r.transpose() << " rhat: " << r_hat.transpose() << " dot: " << rdot << " rcross: " << rcross << " dist: " << dist << " direction: " << query.direction.transpose() << std::endl;
                // }
                if (query.m.norm() > 1e-4) {
                    Vec3 tau = torque(bound, query);
                    Vec3 f = force(bound, query);
                    if (f.norm() > 1.5e-3) {
                        dist = 100*tau.normalized().cross(query.direction).norm();
                    } else {
                        dist = 100*tau.normalized().cross(query.direction).norm();
                    }
                    
                    // std::cout << "\n\tquery p: " << query.point.transpose() <<  " bound p: " << bound.point.transpose()  << " n: " << query.direction.transpose() << " b: " << query.b.transpose() << std::endl;
                    // std::cout << "\t dist: " << dist << " ||tau||: " << tau.norm() << " tau: " << tau.transpose() << " tau hat: " << tau.normalized().transpose() << std::endl;
                    // std::cout << "\t force: " << f.transpose() << " ||f||: " << f.norm() <<  " rhat: " << r_hat.transpose() << " ||r||: " << r.norm() << std::endl;
                }
                
            }
            else {
                dist = r.norm();
                // std::cout << "bound: " << bound.point.transpose() << " query: " << query.point.transpose() << " r: " << r.transpose() << " dist: " << dist << std::endl;
            }
            return dist;
        }

        static Vec3 torque(const Type& bound, const Type& query) {
            Vec3 r = bound.point - query.point;
            Vec3 r_hat = r.normalized();

            double coeff = 66.03*1e-7/std::pow((r.norm() + 25.4)/1000, 3);
            Vec3 b = coeff*(3*r_hat*r_hat.transpose() - Eigen::Matrix3d::Identity())*query.b;
            Vec3 tau = query.m.cross(b);
            return tau;            
        }

        static Vec3 force(const Type& bound, const Type& query) {
            Vec3 r = bound.point - query.point;
            Vec3 r_hat = r.normalized();
            double force_coeff = (66.03*3*1e-7)/(std::pow((r.norm() + 25.4)/1000,4));
            Vec3 force = (query.b*r_hat.transpose() + r_hat*query.b.transpose() + (r_hat.transpose()*query.b)*(Eigen::Matrix3d::Identity() - 5*r_hat*r_hat.transpose())).transpose()*query.m;  
            return force;          
        }
    };
};



namespace unc::robotics::snp {

using NormalSpace = nigh::metric::Space<BoundPoint, nigh::metric::LP<4>>;
using BoundPair = std::pair<BoundNode, RealNum>;
using BoundNN =
    nigh::Nigh<BoundNode, NormalSpace, BoundKey, nigh::Concurrent, nigh::KDTreeBatch<>>;
using BoundNNWithMask = std::pair<bool, BoundNN*>;

class BoundEnvironment {
  public:
    BoundEnvironment(const Affine& ijk_to_ras=Affine::Identity());
    ~BoundEnvironment();

    void SetIjkToRasAffine(const Affine& ijk_to_ras);
    Affine IjkToRasAffine() const;

    void SetBoundaryImageSize(const IdxPoint& size);
    void SetBoundaryImageSize(const Idx size_x, const Idx size_y, const Idx size_z);
    IdxPoint BoundaryImageSize() const;

    bool CreateNewNN(const Str boundary_name="default");
    bool RemoveNN(const Str boundary_name);
    bool EnableNN(const Str boundary_name);
    bool DisableNN(const Str boundary_name);
    std::vector<Str> ListAllNN() const;
    std::vector<Str> ListActiveNN() const;

    void GenerateEmptyBoundaryImage(const Idx size_x, const Idx size_y, const Idx size_z);
    bool ConstructBoundaryEnvironmentFromFile(const Str file_name);


    Vec3 IjkToRas(const IdxPoint& p) const;
    Vec3 IjkToRas(const Idx& i, const Idx& j, const Idx& k) const;

    IdxPoint RasToIjk(const Vec3& p) const;
    IdxPoint RasToIjk(const RealNum& r, const RealNum& a, const RealNum& s) const;

    void AddBoundary(const Vec3& p, const Vec3& n, const Str& boundary_name="default");
    void AddBoundary(const IdxPoint& p, const Vec3& n, const Str& boundary_name="default");

    bool IsBoundary(const IdxPoint& p) const;
    bool IsBoundary(const Vec3& p) const;
    bool IsBoundaryCenter(const Vec3& p) const;

    IdxPoint NearestBoundary(const IdxPoint& p, const Vec3& n) const;
    IdxPoint NearestBoundary(const Vec3& p, const Vec3& n) const;

    std::pair<Vec3, RealNum> NearestBoundaryCenter(const Vec3& p, const Vec3& n, const Vec3& m, const Vec3& b) const;
    std::pair<Vec3, RealNum> NearestBoundaryCenter(const Vec3& p, const Vec3& n, const Str& boundary_name) const;

    RealNum DistanceToBoundaryCenter(const Vec3& p, const Vec3& n) const;
    RealNum DistanceToBoundaryCenter(const Vec3& p, const Vec3& n, const Str& boundary_name) const;

    bool WithinBoundaryImage(const Vec3& p) const;
    bool WithinBoundaryImage(const IdxPoint& p) const;

    RealNum VoxelRadius() const;

    void ClearWhiteList();
    void SetWhiteList(bool flag);
    void AddToWhiteList(const Vec3& p, const RealNum r);
    bool InWhiteListArea(const Vec3& p) const;


    void SetMinDist(const RealNum dist);
    RealNum MinDist() const;

    bool CollisionFree(const Vec3& p) const;
    bool CollisionFree(const Vec3& p, const Str& boundary_name) const;

    bool BoundariesEnabled() const;
    void EnableBoundaries();
    void DisableBoundaries();

    bool HitDetectionsEnabled() const;
    void EnableHitDetection();
    void DisableHitDetection();

    void SaveRasPtc(const Str& file_name) const;

    void SetWorkspace(const bool value=true);
    void SetWorkspace(const Idx& x, const Idx& y, const Idx& z, const bool value=true);
    bool Workspace(const Idx& x, const Idx& y, const Idx& z) const;

  private:
    Affine ijk_to_ras_{Affine::Identity()};
    Affine ras_to_ijk_{Affine::Identity()};
    RealNum voxel_rad_{0};
    IdxPoint bound_size_{IdxPoint(0, 0, 0)};
    std::map<Str, SizeType> bound_idx_;
    RealNum nn_search_rad_{50};

    RealNum min_dist_to_bound_{0};
    BoolArray3 workspace_;

    bool use_white_list_{false};
    std::vector<std::pair<Vec3, RealNum>> white_list_;

    bool use_bounds_{true};
    bool use_hit_detection_{false};


    std::map<Str, BoundNNWithMask> all_nns_;
};

using BoundPtr = std::shared_ptr<BoundEnvironment>;

} // namespace unc::robotics::snp

#include "impl/boundary_environment.hpp"

#endif // SNP_BOUND_ENVIRONMENG_H
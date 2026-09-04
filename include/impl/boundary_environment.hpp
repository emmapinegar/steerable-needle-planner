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

#include <iostream>
#include <fstream>

#include "needle_utils.h"

namespace unc::robotics::snp {

/**
 * Creates an instance of BoundEnvironment. 
 * 
 * @param ijk_to_ras: transformation matrix from image coordinates to world coordinates
 */
BoundEnvironment::BoundEnvironment(const Affine& ijk_to_ras) {
    this->SetIjkToRasAffine(ijk_to_ras);
}

/**
 * Destructs an instance of BoundEnvironment. 
 */
BoundEnvironment::~BoundEnvironment() {
    for (auto const& [name, nn] : all_nns_) {
        if (nn.second) {
            delete nn.second;
        }
    }
}

/**
 * Sets the transformation between image coordinates and world coordinates.
 * 
 * @param ijk_to_ras: transformation matrix from image coordinates to world coordinates
 */
void BoundEnvironment::SetIjkToRasAffine(const Affine& ijk_to_ras) {
    ijk_to_ras_ = ijk_to_ras;
    ras_to_ijk_ = ijk_to_ras.inverse();
    voxel_rad_ = 0.5*Vec3(ijk_to_ras_(0, 0), ijk_to_ras_(1, 1), ijk_to_ras_(2, 2)).norm();
}

/**
 * Gets the transformation between image coordinates and world coordinates.
 * 
 * @returns Affine transformation matrix from image coordinates to world coordinates
 */
Affine BoundEnvironment::IjkToRasAffine() const {
    return ijk_to_ras_;
}

/**
 * Sets the image size.
 * 
 * @param size: dimensions of the image
 */
void BoundEnvironment::SetBoundaryImageSize(const IdxPoint& size) {
    this->SetBoundaryImageSize(size[0], size[1], size[2]);
}

/**
 * Sets the image size.
 * 
 * @param size_x: size of image in the x dimension
 * @param size_y: size of image in the y dimension
 * @param size_z: size of image in the z dimension
 */
void BoundEnvironment::SetBoundaryImageSize(const Idx size_x, const Idx size_y, const Idx size_z) {
    bound_size_[0] = size_x;
    bound_size_[1] = size_y;
    bound_size_[2] = size_z;

    workspace_.resize(boost::extents[size_x][size_y][size_z]);
}

/**
 * Gets the image size.
 * 
 * @returns IdxPoint image size [x, y, z]
 */
IdxPoint BoundEnvironment::BoundaryImageSize() const {
    return bound_size_;
}

/**
 * Creates a new nearest neighbors from image if one has not been created for the image.
 * 
 * @param boundary_name: name of the image to use to create new nearest neighbors
 * 
 * @returns bool true if nearest neighbors was created, false otherwise
 */
bool BoundEnvironment::CreateNewNN(const Str boundary_name) {
    if (all_nns_.find(boundary_name) != all_nns_.end()) {
        std::cerr << "Boundary with name " << boundary_name << " already exists!" << std::endl;
        return false;
    }

    BoundNNWithMask tmp;
    tmp.first = true;
    tmp.second = new BoundNN();
    all_nns_.insert(std::pair<Str, BoundNNWithMask>(boundary_name, tmp));

    bound_idx_[boundary_name] = 0;
    return true;
}

/**
 * Removes the nearest neighbors generated using the specified image if it exists.
 * 
 * @param boundary_name: name of the image to remove from the nearest neighbors
 * 
 * @returns bool true if the nearest neighbors for the image was removed, false if it didn't exist in the first place
 */
bool BoundEnvironment::RemoveNN(const Str boundary_name) {
    auto search = all_nns_.find(boundary_name);

    if (search == all_nns_.end()) {
        std::cerr << "Boundary with name " << boundary_name << " does not exist!" << std::endl;
        return false;
    }

    delete search->second.second;
    all_nns_.erase(boundary_name);
    bound_idx_.erase(boundary_name);
    return true;
}

/**
 * Enables the nearest neighbors for the specified image if it exists. 
 * 
 * @param boundary_name: name of the image to enable nearest neighbors for
 * 
 * @returns bool true if the nearest neighbors for the image was enabled, false if it didn't exist in the first place
 */
bool BoundEnvironment::EnableNN(const Str boundary_name) {
    auto search = all_nns_.find(boundary_name);

    if (search == all_nns_.end()) {
        std::cerr << "Boundary with name " << boundary_name << " does not exist!" << std::endl;
        return false;
    }

    search->second.first = true;
    return true;
}

/**
 * Disables the nearest neighbors for the specified image if it exists. 
 * 
 * @param boundary_name: name of the image to disable nearest neighbors for
 * 
 * @returns bool true if the nearest neighbors for the image was disabled, false if it didn't exist in the first place
 */
bool BoundEnvironment::DisableNN(const Str boundary_name) {
    auto search = all_nns_.find(boundary_name);

    if (search == all_nns_.end()) {
        std::cerr << "Boundary with name " << boundary_name << " does not exist!" << std::endl;
        return false;
    }

    search->second.first = false;
    return true;
}

/**
 * Lists all the nearest neighbors that have been added for different images.
 * 
 * @returns vector<Str> names of images with nearest neighbors that have been added
 */
std::vector<Str> BoundEnvironment::ListAllNN() const {
    std::vector<Str> names;

    for (auto const& [name, nn] : all_nns_) {
        names.push_back(name);
    }

    return names;
}

/**
 * Lists all the nearest neighbors that have been added and activated for different images.
 * 
 * @returns vector<Str> names of images with nearest neighbors that have been added and activated
 */
std::vector<Str> BoundEnvironment::ListActiveNN() const {
    std::vector<Str> names;

    for (auto const& [name, nn] : all_nns_) {
        if (nn.first) {
            names.push_back(name);
        }
    }

    return names;
}

/**
 * Sets the image to the specified size with no obstacles.
 * 
 * @param size_x: size of image in the x dimension
 * @param size_y: size of image in the y dimension
 * @param size_z: size of image in the z dimension
 */
void BoundEnvironment::GenerateEmptyBoundaryImage(const Idx size_x, const Idx size_y, const Idx size_z) {
    this->SetBoundaryImageSize(size_x, size_y, size_z);
}

/**
 * Creates an environment using the provided file.
 * 
 * @param file_name: file containing the environment 
 * 
 * @returns bool true if the environment was successfully created, false otherwise
 * @throws runtime_error if the file can't be opened
 */
bool BoundEnvironment::ConstructBoundaryEnvironmentFromFile(const Str file_name) {
    std::ifstream fin;
    fin.open(file_name);

    char delimiter = ' ';
    if (file_name.compare(file_name.size() - 4, 4, ".csv") == 0) {
        delimiter = ',';
    }

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    Str field;

    Str line;
    Affine ijk_to_ras;

    for (auto i = 0; i < 4; ++i) {
        if (std::getline(fin, line)) {
            std::istringstream s(line);

            for (auto j = 0; j < 4; ++j) {
                std::getline(s, field, delimiter);
                ijk_to_ras(i, j) = std::stod(field);
            }
        }
        else {
            std::cout << "Wrong format for ras_to_ijk affine!" << std::endl;
            return false;
        }
    }

    this->SetIjkToRasAffine(ijk_to_ras);

    IdxPoint size;

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        for (auto i = 0; i < 3; ++i) {
            std::getline(s, field, delimiter);
            size[i] = std::stoi(field);
        }
    }
    else {
        std::cout << "Wrong format for boundary size!" << std::endl;
        return false;
    }

    this->SetBoundaryImageSize(size);

    this->CreateNewNN("default");
    IdxPoint bound;
    Vec3 zero(0,0,0);

    while (std::getline(fin, line)) {
        std::istringstream s(line);

        for (auto i = 0; i < 3; ++i) {
            std::getline(s, field, delimiter);
            bound[i] = std::stoi(field);
        }

        this->AddBoundary(bound, zero, "default");
        if (bound_idx_["default"] % 1000 == 0)
        {
            std::cout << "\rLoaded boundary points: " << bound_idx_["default"] << std::flush;
        }

    }
    std::cout << "\rLoaded boundary points: " << bound_idx_["default"] << std::flush;
    std::cout << std::endl;
    return true;
}

/**
 * Calculates the world (ras) coordinates for the given point in image (ijk) coordinates.
 * 
 * @param p: ijk coordinate to convert
 * 
 * @returns Vec3 world coordinates for the provided image coordinates
 */
Vec3 BoundEnvironment::IjkToRas(const IdxPoint& p) const {
    return ijk_to_ras_*Vec3(p[0], p[1], p[2]);
}

/**
 * Calculates the world (ras) coordinates for the given point in image (ijk) coordinates.
 * 
 * @param i: i coordinate to convert
 * @param j: j coordinate to convert
 * @param k: k coordinate to convert
 * 
 * @returns Vec3 world coordinates for the provided image coordinates
 */
Vec3 BoundEnvironment::IjkToRas(const Idx& i, const Idx& j, const Idx& k) const {
    return ijk_to_ras_*Vec3(i, j, k);
}

/**
 * Calculates the image (ijk) coordinates for the given point in world (ras) coordinates.
 * 
 * @param p: ras coordinate to convert
 * 
 * @returns IdxPoint image coordinates for the provided world coordinates
 */
IdxPoint BoundEnvironment::RasToIjk(const Vec3& p) const {
    Vec3 rough = ras_to_ijk_*p;

    if (rough[0] < 0 || rough[1] < 0 || rough[2] < 0) {
        std::cerr << "Negative index! Point: "
                  << p.transpose()
                  << " to "
                  << rough.transpose()
                  << std::endl;
        getchar();
    }

    return IdxPoint(round(rough[0]), round(rough[1]), round(rough[2]));
}

/**
 * Calculates the image (ijk) coordinates for the given point in world (ras) coordinates.
 * 
 * @param r: r coordinate to convert
 * @param a: a coordinate to convert
 * @param s: s coordinate to convert
 * 
 * @returns IdxPoint image coordinates for the provided world coordinates
 */
IdxPoint BoundEnvironment::RasToIjk(const RealNum& r, const RealNum& a, const RealNum& s) const {
    return this->RasToIjk(Vec3(r, a, s));
}

/**
 * Adds obstacle to the environment.
 * 
 * @param p: location of obstacle in world (ras) coordinates
 * @param boundary_name: name of the image the obstacle should be added to for nearest neighbors 
 */
void BoundEnvironment::AddBoundary(const Vec3& p, const Vec3& n, const Str& boundary_name) {
    auto search = all_nns_.find(boundary_name);

    if (search == all_nns_.end()) {
        throw std::runtime_error("Boundary with name " + boundary_name + " does not exist! Cannot add boundary!");
    }

    BoundNN* nn = search->second.second;
    BoundPoint query{p, n};
    auto nearest = nn->nearest(query);
    // std::cout << "p: " << p.transpose() << " point: " << query.point.transpose() << " nearest: " << nearest->first.bound.point.transpose() << std::endl;

    if (!nearest || (nearest->first.bound.point - p).norm() > EPS) {
        nn->insert(BoundNode(bound_idx_[boundary_name]++, p, n));
    }
}

/**
 * Adds boundary to the environment.
 * 
 * @param p: location of obstacle in image (ijk) coordinates
 * @param boundary_name: name of the image the obstacle should be added to for nearest neighbors 
 */
void BoundEnvironment::AddBoundary(const IdxPoint& p, const Vec3& n, const Str& boundary_name) {
    if (!this->WithinBoundaryImage(p)) {
        throw std::runtime_error("Trying to add an boundary outside the boundary region.");
    }

    auto ras_p = this->IjkToRas(p);
    this->AddBoundary(ras_p, n, boundary_name);
}

/**
 * Checks if the provided coordinates are an obstacle.
 * 
 * @param p: image (ijk) coordinates of the point to check for an obstacle
 * 
 * @returns bool true if the point is an obstacle center or the point is beyond the bounds of the image, false otherwise
 */
bool BoundEnvironment::IsBoundary(const IdxPoint& p) const {
    if (this->WithinBoundaryImage(p)) {
        return this->IsBoundaryCenter(this->IjkToRas(p));
    }

    std::cerr << "Out of boundary region!" << std::endl;
    return true;
}

/**
 * Checks if the provided coordinates are an obstacle.
 * 
 * @param p: world (ras) coordinates of the point to check for an obstacle
 * 
 * @returns bool true if the point is an obstacle center, false otherwise
 */
bool BoundEnvironment::IsBoundary(const Vec3& p) const {
    const Vec3 center = this->IjkToRas(this->RasToIjk(p));

    return this->IsBoundaryCenter(center);
}

/**
 * Checks if the provided coordinates are the center of an obstacle. 
 * 
 * @param p: world (ras) coordinates of the point to check for an obstacle
 * 
 * @returns bool true if the point is within EPS of an obstacle center
 */
bool BoundEnvironment::IsBoundaryCenter(const Vec3& p) const {
    return (this->DistanceToBoundaryCenter(p, Vec3(0,0,0)) < EPS);
}

/**
 * Gets the nearest obstacle to the specified coordinates. 
 * 
 * @param p: image (ijk) coordinates of the point to find the closest obstacle
 * 
 * @returns IdxPoint closest obstacle to the point 
 */
IdxPoint BoundEnvironment::NearestBoundary(const IdxPoint& p, const Vec3& n) const {
    return this->NearestBoundary(this->IjkToRas(p), n);
}

/**
 * Gets the nearest obstacle to the specified coordinates. 
 * 
 * @param p: world (ras) coordinates of the point to find the closest obstacle
 * 
 * @returns IdxPoint closest obstacle to the point 
 */
IdxPoint BoundEnvironment::NearestBoundary(const Vec3& p, const Vec3& n) const {
    auto [point, min_dist] = this->NearestBoundaryCenter(p, n, Vec3(0,0,0), Vec3(0,0,0));

    if(min_dist == R_INF) {
        return IdxPoint(0, 0, 0);
    }

    return this->RasToIjk(point);
}

/**
 * Gets the nearest obstacle center to the specified coordinates. 
 * 
 * @param p: world (ras) coordinates of the point to find the closest obstacle
 * 
 * @returns pair<Vec3, RealNum> center of the closest obstacle, the distance between the point and obstacle
 */
std::pair<Vec3, RealNum> BoundEnvironment::NearestBoundaryCenter(const Vec3& p, const Vec3& n, const Vec3& m, const Vec3& b) const {
    RealNum min_dist = R_INF;
    Vec3 point(0, 0, 0);

    for (auto const& [name, nn] : all_nns_) {
        if (nn.first) {
            BoundPoint q_point{p, n, m, b};
            auto p_nearest = nn.second->nearest(q_point);

            if (p_nearest && p_nearest->second < min_dist) {
                min_dist = p_nearest->second;
                point = p_nearest->first.bound.point;
            }
        }
    }

    return {point, min_dist};
}

/**
 * Gets the nearest obstacle center to the specified coordinates. 
 * 
 * @param p: world (ras) coordinates of the point to find the closest obstacle
 * @param boundary_name: name of the image to search for the nearest obstacle
 * 
 * @returns pair<Vec3, RealNum> center of the closest obstacle, the distance between the point and obstacle
 */
std::pair<Vec3, RealNum> BoundEnvironment::NearestBoundaryCenter(const Vec3& p, const Vec3& n,
        const Str& boundary_name) const {
    auto search = all_nns_.find(boundary_name);

    if (search == all_nns_.end()) {
        throw std::runtime_error("Boundary with name " + boundary_name +
                                 "does not exist! Cannot query obstacle center!");
    }

    auto nn = search->second.second;
    auto p_nearest = nn->nearest(BoundPoint{p,n});

    if (!p_nearest) {
        return {Vec3(0, 0, 0), R_INF};
    }

    return {p_nearest->first.bound.point, p_nearest->second};
}

/**
 * Calculates the distance from the point to the nearest obstacle center.
 * 
 * @param p: world (ras) coordinates of the point to find the closest obstacle
 * 
 * @returns RealNum  minimum of the distance between point and nearest obstacle center and the nearest neighbors search radius
 */
RealNum BoundEnvironment::DistanceToBoundaryCenter(const Vec3& p, const Vec3& n) const {
    auto [point, min_dist] = this->NearestBoundaryCenter(p, n, Vec3(0,0,0), Vec3(0,0,0));

    if(min_dist == R_INF) {
        return nn_search_rad_;
    }

    return fmin(min_dist, nn_search_rad_);
}

/**
 * Calculates the distance from the point to the nearest obstacle center.
 * 
 * @param p: world (ras) coordinates of the point to find the closest obstacle
 * @param boundary_name: name of the image to search for the nearest obstacle
 * 
 * @returns RealNum  minimum of the distance between point and nearest obstacle center and the nearest neighbors search radius
 */
RealNum BoundEnvironment::DistanceToBoundaryCenter(const Vec3& p, const Vec3& n, const Str& boundary_name) const {
    auto [point, min_dist] = this->NearestBoundaryCenter(p, n, boundary_name);

    if(min_dist == R_INF) {
        return nn_search_rad_;
    }

    return fmin(min_dist, nn_search_rad_);
}

/**
 * Checks if the point is within the bounds of the image.
 * 
 * @param p: world (ras) coordinates of the point to check 
 * 
 * @returns bool true if the converted ijk coordinates are within the image bounds, false otherwise
 */
bool BoundEnvironment::WithinBoundaryImage(const Vec3& p) const {
    Vec3 rough = ras_to_ijk_*p;

    if (rough[0] < 0 || rough[1] < 0 || rough[2] < 0) {
        return false;
    }

    if (round(rough[0]) < bound_size_[0]
            && round(rough[1]) < bound_size_[1]
            && round(rough[2]) < bound_size_[2]) {
        return true;
    }

    return false;
}

/**
 * Checks if the point is within the bounds of the image.
 * 
 * @param p: image (ijk) coordinates of the point to check 
 * 
 * @returns bool true if the converted ijk coordinates are within the image bounds, false otherwise
 */
bool BoundEnvironment::WithinBoundaryImage(const IdxPoint& p) const {
    if (p[0] < bound_size_[0] && p[1] < bound_size_[1] && p[2] < bound_size_[2]) {
        return true;
    }

    return false;
}

/**
 * Gets the voxel radius for the environment.
 * 
 * @returns RealNum radius of voxel
 */
RealNum BoundEnvironment::VoxelRadius() const {
    return voxel_rad_;
}

/**
 * Removes all "white list" components.
 */
void BoundEnvironment::ClearWhiteList() {
    auto n = white_list_.size();
    white_list_.clear();

    std::cout << "Removed " << n << " white list components." << std::endl;
}

/**
 * Sets the "white list" flag to determine if it is used.
 * 
 * @param flag: true if "white list" should be used, false otherwise
 */
void BoundEnvironment::SetWhiteList(bool flag) {
    use_white_list_ = flag;
}

/**
 * Checks if the provided coordinates are in the "white list" area.
 * 
 * @param p: world (ras) coordinates of the point to check
 * 
 * @returns bool true if the cooridnates are within the "white list" area, false otherwise
 */
bool BoundEnvironment::InWhiteListArea(const Vec3& p) const {
    for (const auto ball : white_list_) {
        if ((p - ball.first).norm() < ball.second) {
            return true;
        }
    }

    return false;
}

/**
 * Adds the provided coordinates to the "white list" with the provided radius.
 * 
 * @param p: world (ras) coordinates to add to "white list"
 * @param r: radius of area associated with this point
 */
void BoundEnvironment::AddToWhiteList(const Vec3& p, const RealNum r) {
    white_list_.emplace_back(p, r);
}

/**
 * Sets the minimum distance to obstacles.
 * 
 * @param dist: the new minimum distance from obstacles
 */
void BoundEnvironment::SetMinDist(const RealNum dist) {
    min_dist_to_bound_ = dist;
}

/**
 * Gets the minimum distance to obstacles.
 * 
 * @returns RealNum minimum distance from obstacles
 */
RealNum BoundEnvironment::MinDist() const {
    return min_dist_to_bound_;
}

/**
 * Checks if the provided point is collision free.
 * 
 * @param p: world (ras) coordinates to check for collisions
 * 
 * @returns true if the point is not in collision with obstacles, false otherwise
 * @throws runtime_error if the obstacle name can't be found 
 */
bool BoundEnvironment::CollisionFree(const Vec3& p) const {
    if (!this->WithinBoundaryImage(p)) {
        return false;
    }

    if (!use_bounds_) {
        return true;
    }

    if (use_hit_detection_) {
        return !this->IsBoundary(p);
    }
    Vec3 zero(0,0,0);
    BoundPoint query{p,zero};
    for (auto const& [name, nn] : all_nns_) {
        if (nn.first) {
            auto p_nearest = nn.second->nearest(query);
            if (p_nearest && p_nearest->second < min_dist_to_bound_) {
                if (!this->InWhiteListArea(p_nearest->first.bound.point)) {
                    return false;
                }

                std::vector<BoundPair> nbh;
                auto search = bound_idx_.find(name);

                if (search == bound_idx_.end()) {
                    throw std::runtime_error("Cannot find " + name + " in boundary idx map!");
                }

                nn.second->nearest(nbh, query, search->second, min_dist_to_bound_);

                for (auto n : nbh) {
                    if (!this->InWhiteListArea(n.first.bound.point)) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

/**
 * Checks if the provided point is collision free.
 * 
 * @param p: world (ras) coordinates to check for collisions
 * @param boundary_name: name of the image to search for obstacles
 * 
 * @returns true if the point is not in collision with obstacles, false otherwise
 * @throws runtime_error if the obstacle name can't be found 
 */
bool BoundEnvironment::CollisionFree(const Vec3& p, const Str& boundary_name) const {
    if (!this->WithinBoundaryImage(p)) {
        return false;
    }

    if (!use_bounds_) {
        return true;
    }

    if (use_hit_detection_) {
        throw std::runtime_error("No implementation for hit detection for a specific boundary!");
    }

    auto search = all_nns_.find(boundary_name);

    if (search == all_nns_.end()) {
        throw std::runtime_error("Boundary with name " + boundary_name +
                                 "does not exist! Cannot query collision!");
    }
    Vec3 zero(0,0,0);
    BoundPoint query{p,zero};
    auto nn = search->second;
    auto p_nearest = nn.second->nearest(query);

    if (p_nearest && p_nearest->second < min_dist_to_bound_) {
        if (!this->InWhiteListArea(p_nearest->first.bound.point)) {
            return false;
        }

        std::vector<BoundPair> nbh;
        auto search_idx = bound_idx_.find(boundary_name);

        if (search_idx == bound_idx_.end()) {
            throw std::runtime_error("Cannot find " + boundary_name + " in boundary idx map!");
        }

        nn.second->nearest(nbh, query, search_idx->second, min_dist_to_bound_);

        for (auto n : nbh) {
            if (!this->InWhiteListArea(n.first.bound.point)) {
                return false;
            }
        }
    }

    return true;
}

/**
 * Gets if obstacles are enabled in the environment.
 * 
 * @returns bool true if obstacles are enabled, false otherwise
 */
bool BoundEnvironment::BoundariesEnabled() const {
    return use_bounds_;
}

/**
 * Enables obstacles in the environment. 
 */
void BoundEnvironment::EnableBoundaries() {
    use_bounds_ = true;
}

/**
 * Disables obstacles in the environment. 
 */
void BoundEnvironment::DisableBoundaries() {
    use_bounds_ = false;
}

/**
 * Gets if hit detections are enabled.
 * 
 * @returns bool true if hit detections are enabled, false otherwise
 */
bool BoundEnvironment::HitDetectionsEnabled() const {
    return use_hit_detection_;
}

/**
 * Enables hit detection in the environment.
 */
void BoundEnvironment::EnableHitDetection() {
    use_hit_detection_ = true;
}

/**
 * Disables hit detection in the environment.
 */
void BoundEnvironment::DisableHitDetection() {
    use_hit_detection_ = false;
}

/**
 * Saves the obstacles in world coordinates as a point cloud.
 * 
 * @param file_name: file ot save the obstacle point cloud to
 * 
 * @throws runtime_error if the specified file cannot be opened
 */
void BoundEnvironment::SaveRasPtc(const Str& file_name) const {
    std::ofstream fout;
    fout.open(file_name);

    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    std::cout << "Saving ras point cloud..." << std::endl;

    SizeType count = 0;

    for (auto const& [name, nn] : all_nns_) {
        auto ptc = nn.second->list();

        for (const auto& p : ptc) {
            const Vec3& point = p.bound.point;

            fout << point[0] << " " << point[1] << " " << point[2] << std::endl;
        }

        count += ptc.size();
    }

    std::cout << "Saved " << count << " points to " << file_name << std::endl;
}

/**
 * Sets the workspace elements to value.
 * 
 * @param value: bool value all workspace elements are set to
 */
void BoundEnvironment::SetWorkspace(const bool value) {
    std::fill(workspace_.data(), workspace_.data() + workspace_.num_elements(), value);
}

/**
 * Sets the workspace element to value.
 * 
 * @param x: x coordinate of workspace element
 * @param y: y coordinate of workspace element
 * @param z: z coordinate of workspace element
 * @param value: bool value of workspace element
 */
void BoundEnvironment::SetWorkspace(const Idx& x, const Idx& y, const Idx& z, const bool value) {
    workspace_[x][y][z] = value;
}

/**
 * Gets the value of the workspace element.
 * 
 * @param x: x coordinate of workspace element
 * @param y: y coordinate of workspace element
 * @param z: z coordinate of workspace element
 * 
 * @returns bool value of workspace element
 */
bool BoundEnvironment::Workspace(const Idx& x, const Idx& y, const Idx& z) const {
    return workspace_[x][y][z];
}

} // namespace unc::robotics::snp
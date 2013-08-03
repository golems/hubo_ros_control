/*
 * Copyright (c) 2013, Drexel DARPA Robotics Challenge team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Jim Mainprice (WPI) */

#ifndef HUBO_TRAJECTORY_HPP
#define HUBO_TRAJECTORY_HPP

#include <vector>
#include <string>
#include <iosfwd>
#include <sstream>
#include <map>

namespace Hubo
{
typedef std::vector<double> Vector;
typedef std::pair<double,Vector> Milestone;

typedef std::vector<Milestone> Milestones;
typedef Milestones::const_iterator MilestonePtr;

typedef std::map<std::string,int> JointMap;
typedef std::map<std::string,int>::iterator JointMapPtr;

//! piecewise linear trajectory
class Trajectory
{
public:
    Trajectory();
    bool load_from_file( const std::string& filename, double frequency );
    void push_back( const Milestone& q );
    void clear();
    Vector get_config_at_time(double t) const;
    double get_length() const;
    int get_number_of_milestones() const { return milestones_.size(); }
    int empty() const { return milestones_.empty(); }
private:
    Vector interpolate(  const Vector& a, const Vector& b, double u ) const;
    void set_milestones_from_path( const std::vector<Vector>& path, double dt );
    Milestones milestones_;
};

//! function that converts text to num
//! returns false if fails
template <class T>
bool convert_text_to_num(T& t,
                         const std::string& s,
                         std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

}

#endif // OR_TRAJECTORY_HPP

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/** \author Ioan Sucan, E. Gil Jones */

#include "collision_detection/collision_matrix.h"
#include <iomanip>

collision_detection::AllowedCollisionMatrix::AllowedCollisionMatrix(void)
{
}

collision_detection::AllowedCollisionMatrix::AllowedCollisionMatrix(const std::vector<std::string>& names, bool allowed)
{
    for (std::size_t i = 0 ; i < names.size() ; ++i)
        for (std::size_t j = i + 1 ; j < names.size() ; ++j)
            setEntry(names[i], names[j], allowed);
}

collision_detection::AllowedCollisionMatrix::AllowedCollisionMatrix(const AllowedCollisionMatrix& acm)
{
    entries_ = acm.entries_;
    allowed_contacts_ = acm.allowed_contacts_;
}

bool collision_detection::AllowedCollisionMatrix::getAllowedCollision(const std::string& name1, const std::string& name2, DecideContactFn &fn) const
{
    std::map<std::string, std::map<std::string, DecideContactFn> >::const_iterator it1 = allowed_contacts_.find(name1);
    if (it1 == allowed_contacts_.end())
        return false;
    std::map<std::string, DecideContactFn>::const_iterator it2 = it1->second.find(name2);
    if (it2 == it1->second.end())
        return false;
    fn = it2->second;
    return true;
}

bool collision_detection::AllowedCollisionMatrix::getAllowedCollision(const std::string& name1, const std::string& name2, AllowedCollision::Type& allowed_collision) const
{
    std::map<std::string, std::map<std::string, AllowedCollision::Type> >::const_iterator it1 = entries_.find(name1);
    if (it1 == entries_.end())
        return false;
    std::map<std::string, AllowedCollision::Type>::const_iterator it2 = it1->second.find(name2);
    if (it2 == it1->second.end())
        return false;
    allowed_collision = it2->second;
    return true;
}

bool collision_detection::AllowedCollisionMatrix::hasEntry(const std::string& name1, const std::string& name2) const
{
    std::map<std::string, std::map<std::string, AllowedCollision::Type> >::const_iterator it1 = entries_.find(name1);
    if (it1 == entries_.end())
        return false;
    std::map<std::string, AllowedCollision::Type>::const_iterator it2 = it1->second.find(name2);
    if (it2 == it1->second.end())
        return false;
    return true;
}

void collision_detection::AllowedCollisionMatrix::setEntry(const std::string &name1, const std::string &name2, bool allowed)
{
    const AllowedCollision::Type v = allowed ? collision_detection::AllowedCollision::ALWAYS : collision_detection::AllowedCollision::NEVER;
    entries_[name1][name2] = entries_[name2][name1] = v;

    // remove boost::function pointers, if any
    std::map<std::string, std::map<std::string, DecideContactFn> >::iterator it = allowed_contacts_.find(name1);
    if (it != allowed_contacts_.end())
    {
        std::map<std::string, DecideContactFn>::iterator jt = it->second.find(name2);
        if (jt != it->second.end())
            it->second.erase(jt);
    }
    it = allowed_contacts_.find(name2);
    if (it != allowed_contacts_.end())
    {
        std::map<std::string, DecideContactFn>::iterator jt = it->second.find(name1);
        if (jt != it->second.end())
            it->second.erase(jt);
    }
}

void collision_detection::AllowedCollisionMatrix::setEntry(const std::string& name1, const std::string& name2, const DecideContactFn &fn)
{
    entries_[name1][name2] = entries_[name2][name1] = AllowedCollision::CONDITIONAL;
    allowed_contacts_[name1][name2] = allowed_contacts_[name2][name1] = fn;
}

void collision_detection::AllowedCollisionMatrix::removeEntry(const std::string& name)
{
    entries_.erase(name);
    allowed_contacts_.erase(name);
    for (std::map<std::string, std::map<std::string, AllowedCollision::Type> >::iterator it = entries_.begin() ; it != entries_.end() ; ++it)
        it->second.erase(name);
    for (std::map<std::string, std::map<std::string, DecideContactFn> >::iterator it = allowed_contacts_.begin() ; it != allowed_contacts_.end() ; ++it)
        it->second.erase(name);
}

void collision_detection::AllowedCollisionMatrix::removeEntry(const std::string& name1, const std::string &name2)
{
    std::map<std::string, std::map<std::string, AllowedCollision::Type> >::iterator jt = entries_.find(name1);
    if (jt != entries_.end())
    {
        std::map<std::string, AllowedCollision::Type>::iterator it = jt->second.find(name2);
        if (it != jt->second.end())
            jt->second.erase(it);
    }
    jt = entries_.find(name2);
    if (jt != entries_.end())
    {
        std::map<std::string, AllowedCollision::Type>::iterator it = jt->second.find(name1);
        if (it != jt->second.end())
            jt->second.erase(it);
    }

    std::map<std::string, std::map<std::string, DecideContactFn> >::iterator it = allowed_contacts_.find(name1);
    if (it != allowed_contacts_.end())
    {
        std::map<std::string, DecideContactFn>::iterator jt = it->second.find(name2);
        if (jt != it->second.end())
            it->second.erase(jt);
    }
    it = allowed_contacts_.find(name2);
    if (it != allowed_contacts_.end())
    {
        std::map<std::string, DecideContactFn>::iterator jt = it->second.find(name1);
        if (jt != it->second.end())
            it->second.erase(jt);
    }
}

void collision_detection::AllowedCollisionMatrix::setEntry(const std::string& name, const std::vector<std::string>& other_names, bool allowed)
{
    for (std::size_t i = 0 ; i < other_names.size() ; ++i)
        if (other_names[i] != name)
            setEntry(other_names[i], name, allowed);
}

void collision_detection::AllowedCollisionMatrix::setEntry(const std::vector<std::string>& names1, const std::vector<std::string>& names2, bool allowed)
{
    for (std::size_t i = 0 ; i < names1.size() ; ++i)
        setEntry(names1[i], names2, allowed);
}

void collision_detection::AllowedCollisionMatrix::setEntry(const std::string& name, bool allowed)
{
    for (std::map<std::string, std::map<std::string, AllowedCollision::Type> >::iterator it = entries_.begin() ; it != entries_.end() ; ++it)
        if (name != it->first)
            setEntry(name, it->first, allowed);
}

void collision_detection::AllowedCollisionMatrix::setEntry(bool allowed)
{
    const AllowedCollision::Type v = allowed ? collision_detection::AllowedCollision::ALWAYS : collision_detection::AllowedCollision::NEVER;
    for (std::map<std::string, std::map<std::string, AllowedCollision::Type> >::iterator it1 = entries_.begin() ; it1 != entries_.end() ; ++it1)
        for (std::map<std::string, AllowedCollision::Type>::iterator it2 = it1->second.begin() ; it2 != it1->second.end() ; ++it2)
            it2->second = v;
}

void collision_detection::AllowedCollisionMatrix::clear(void)
{
    entries_.clear();
    allowed_contacts_.clear();
}

void collision_detection::AllowedCollisionMatrix::getAllEntryNames(std::vector<std::string>& names) const
{
    names.clear();
    for (std::map<std::string, std::map<std::string, AllowedCollision::Type> >::const_iterator it = entries_.begin() ; it != entries_.end() ; ++it)
        names.push_back(it->first);
}

void collision_detection::AllowedCollisionMatrix::print(std::ostream& out) const
{
    std::vector<std::string> names;
    getAllEntryNames(names);
    std::sort(names.begin(), names.end());

    std::size_t L = 4;
    for (std::size_t i = 0 ; i < names.size() ; ++i)
    {
        std::size_t l = names[i].length();
        if (l > L)
            L = l;
    }
    ++L;

    for (std::size_t i = 0 ; i < names.size() ; ++i)
    {
        out << std::setw(L) << names[i];
        out << " | ";
        for (std::size_t j = 0 ; j < names.size() ; ++j)
        {
            AllowedCollision::Type type;
            bool found = getAllowedCollision(names[i], names[j], type);
            if (found)
                out << std::setw(3) << (type == AllowedCollision::ALWAYS ? '1' : (type == AllowedCollision::NEVER ? '0' : '?'));
            else
                out << std::setw(3) << '-';
        }
        out << std::endl;
    }
}

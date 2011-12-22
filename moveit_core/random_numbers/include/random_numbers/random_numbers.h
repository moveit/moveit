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

/* Author: Ioan Sucan */

#ifndef RANDOM_NUMBERS_RANDOM_NUMBERS_
#define RANDOM_NUMBERS_RANDOM_NUMBERS_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

namespace random_numbers
{
    /** \brief Random number generation (wrapper for boost). An instance of this class
        cannot be used by multiple threads at once (member functions
        are not const). However, the constructor is thread safe and
        different instances can be used safely in any number of
        threads. It is also guaranteed that all created instances will
        have a "random" random seed. */
    class RandomNumberGenerator
    {
    public:

        /** \brief Constructor. Always sets a "random" random seed */
        RandomNumberGenerator(void);

        /** \brief Generate a random real between 0 and 1 */
        double uniform01(void)
        {
            return uni_();
        }

      /**
       * @brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound)
       * @param lower_bound The lower bound
       * @param upper_bound The upper bound
       */
        double uniformReal(double lower_bound, double upper_bound)
        {
            return (upper_bound - lower_bound) * uni_() + lower_bound;
        }

        /** \brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w) 
         * @param value[4] A four dimensional array in which the computed quaternion will be returned
         */
        void   quaternion(double value[4]);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, each in the range [-pi, pi). The computed value has the order (roll, pitch, yaw) 
         * @param value[3] A three dimensional array in which the computed euler angles will be released
         */
        void   eulerRPY(double value[3]);

    private:

        boost::mt19937                                                           generator_;
        boost::uniform_real<>                                                    uniDist_;
        boost::variate_generator<boost::mt19937&, boost::uniform_real<> >        uni_;

    };

}

#endif

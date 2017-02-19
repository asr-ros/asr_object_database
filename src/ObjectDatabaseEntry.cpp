/**

Copyright (C) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heizmann Heinrich, Heller Florian, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Walter Milena

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "object_database/ObjectDatabaseEntry.h"
#include <ros_uri/ros_uri.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <math.h>
#include "object_database/NormalGenerator.h"

namespace object_database

{
namespace fs = boost::filesystem;

ObjectDatabaseEntry::ObjectDatabaseEntry(ObjectDatabaseRecognizer* recognizerPtr,
                                         const std::string uniqueName, const fs::path path,
                                         const fs::path rvizMeshResourcePath,
                                         const fs::path normalVectorResourcePath) :
    mRecognizerPtr(recognizerPtr),
    mUniqueName(uniqueName),
    mPath(path),
    mRvizMeshResourcePath(rvizMeshResourcePath),
    mNormalVectorResourcePath(normalVectorResourcePath),
    mNormalVectors()
{ }

ObjectDatabaseEntry::ObjectDatabaseEntry(ObjectDatabaseRecognizer *recognizerPtr,
                                         const std::string uniqueName,
                                         const fs::path path, const fs::path rvizMeshResourcePath,
                                         const fs::path normalVectorResourcePath,
                                         const fs::path deviation_resource_path,
                                         const fs::path rotation_invariance_resource_path):
    mRecognizerPtr(recognizerPtr),
    mUniqueName(uniqueName),
    mPath(path),
    mRvizMeshResourcePath(rvizMeshResourcePath),
    mNormalVectorResourcePath(normalVectorResourcePath),
    deviation_resource_path_(deviation_resource_path),
    rotation_invariance_resource_path(rotation_invariance_resource_path),
    mNormalVectors(),
    deviations_(),
    rotation_invariant(false)
{}

ObjectDatabaseRecognizer* ObjectDatabaseEntry::getRecognizer()
{
    return mRecognizerPtr;
}

const std::string ObjectDatabaseEntry::getUniqueName()
{
    return mUniqueName;
}

const fs::path ObjectDatabaseEntry::getPath()
{
    return mPath;
}

const fs::path ObjectDatabaseEntry::getRvizMeshResourcePath()
{
    return mRvizMeshResourcePath;
}

const fs::path ObjectDatabaseEntry::getNormalVectorResourcePath()
{
    return mNormalVectorResourcePath;
}

std::vector<geometry_msgs::Point> ObjectDatabaseEntry::getNormalVectors()
{
    // if there is no path given or the normal vectors are already given
    if (mNormalVectors.size() > 0)
    {
        ROS_DEBUG("Returning cached normal vectors for object '%s'.", this->getUniqueName().c_str());
        return mNormalVectors;
    }
    else if (this->getNormalVectorResourcePath() == fs::path())
    {
        ROS_WARN("No normal vector file found for object '%s'.", this->getUniqueName().c_str());
        return mNormalVectors;
    }

    if (fs::exists(this->getNormalVectorResourcePath()))
    {
        std::ifstream normalVectorFileHandle(this->getNormalVectorResourcePath().c_str());
        if (normalVectorFileHandle.is_open())
        {
            ROS_INFO("Reading normal vector file for object '%s'", this->getUniqueName().c_str());
            std::string normalVectorString;
            std::size_t startSearchPos = 0;
            while (std::getline(normalVectorFileHandle, normalVectorString))
            {
                while (startSearchPos < normalVectorString.size())
                {
                    std::size_t searchPos = normalVectorString.find(';', startSearchPos);
                    if (searchPos == std::string::npos)
                    {
                        searchPos = normalVectorString.size();
                    }

                    // get the coordinate string.
                    std::string coordinateString = normalVectorString.substr(startSearchPos,
                                                                             (searchPos - startSearchPos));

                    std::size_t startValueSearchPos = 0;
                    std::vector<double> coordinateStack;
                    while (startValueSearchPos < coordinateString.size())
                    {
                        std::size_t valueSearchPos = coordinateString.find(',', startValueSearchPos);
                        if (valueSearchPos == std::string::npos)
                        {
                            valueSearchPos = coordinateString.size();
                        }

                        std::string valueString = coordinateString.substr(startValueSearchPos,
                                                                          (valueSearchPos - startValueSearchPos));


                        double coordinateValue = boost::lexical_cast<double>(valueString);
                        coordinateStack.push_back(coordinateValue);

                        startValueSearchPos = valueSearchPos + 1;
                    }

                    if (coordinateStack.size() != 3)
                    {
                        ROS_WARN("Ignoring a coordinate because there were not exactly three values given.");
                        continue;
                    }

                    // create the normal vector and add it
                    geometry_msgs::Point normalVector;
                    normalVector.x = coordinateStack [0];
                    normalVector.y = coordinateStack [1];
                    normalVector.z = coordinateStack [2];
                    mNormalVectors.push_back(normalVector);

                    startSearchPos = searchPos + 1;
                }
            }


            // close file.
            normalVectorFileHandle.close();
            return mNormalVectors;
        }
    }

    //if(fs::exists(this->getRvizMeshResourcePath()) && fs::exists(this->getPath()))
    else if(fs::exists(this->getPath()))
    {
        ROS_INFO("Calculating normal vectors for object '%s'", this->getUniqueName().c_str());
        // calculate the normal vector
        mNormalVectors = NormalGenerator::getNormalVectors(30 * M_PI / 180, 30 * M_PI / 180, 30 * M_PI / 180,
                                                           this->getPath().string() + "/" +
                                                           this->getUniqueName() + ".nv.txt");
    }

    return mNormalVectors;
}

fs::path ObjectDatabaseEntry::getDeviationResourcePath() const
{
    return deviation_resource_path_;
}

std::vector<double> ObjectDatabaseEntry::getDeviationsFromFile()
{
    if (deviations_.size() == 6)
    {
        return deviations_;
    }
    if (fs::exists(getDeviationResourcePath()))
    {
        std::ifstream deviation_file_handle(getDeviationResourcePath().c_str());
        if (deviation_file_handle.is_open())
        {
            ROS_INFO("Reading deviation file for object '%s'", getUniqueName().c_str());
            std::string deviation_string;

            unsigned int line_number = 0;
            while (std::getline(deviation_file_handle, deviation_string))
            {

                line_number++;
                if (boost::starts_with(deviation_string, "#"))
                    continue;
                try
                {
                    deviations_.push_back(std::stod(deviation_string));
                }
                catch (const std::invalid_argument& ia)
                {
                    ROS_ERROR_STREAM("Error by parsing" << getDeviationResourcePath().c_str() <<
                                     " in line " << line_number << ": " << ia.what());
                }
                if(deviations_.size() >= 6)
                    break;
            }
            if (deviations_.size() < 6)
            {
                ROS_ERROR_STREAM("There were only " << deviations_.size()
                                 << "values for the deviation, but 6 are needed");
                deviations_.clear();
            }
            for (double d: deviations_)
                ROS_INFO_STREAM("read deviation:" << d );
            deviation_file_handle.close();
        }
    }
    return deviations_;
}

fs::path ObjectDatabaseEntry::getRotationInvarianceResourcePath() const
{
    return rotation_invariance_resource_path;
}

const bool ObjectDatabaseEntry::getRotationInvarianceFromFile()
{
    // if there is no path given or the normal vectors are already given
    if (rotation_invariant)
    {
        return rotation_invariant;
    }
    else if (this->getRotationInvarianceResourcePath() == fs::path())
    {
        ROS_WARN("No rotation invariance file found for object '%s'.", this->getUniqueName().c_str());
        return rotation_invariant;
    }

    if (fs::exists(this->getRotationInvarianceResourcePath()))
    {
        std::ifstream rotation_invariance_file_handle(this->getRotationInvarianceResourcePath().c_str());
        if (rotation_invariance_file_handle.is_open())
        {
            ROS_INFO("Reading rotation invariance file for object '%s'", this->getUniqueName().c_str());
            std::string rotation_invariance_string;
            std::getline(rotation_invariance_file_handle, rotation_invariance_string);
            rotation_invariant = boost::lexical_cast<bool>(rotation_invariance_string);

            // close file.
            rotation_invariance_file_handle.close();
        }
    }
    return rotation_invariant;
}
}


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

#ifndef OBJECT_DATABASE_CATEGORY_H_
#define OBJECT_DATABASE_CATEGORY_H_


#include <boost/filesystem.hpp>
#include <string>
#include "typedef.h"
#include "ObjectDatabaseEntry.h"
#include "asr_object_database/ConfigurationXMLSchema.h"

namespace object_database
{
using namespace asr_object_database;

namespace fs = boost::filesystem;
/**
 * The FileType used for the FileType.
 */
struct FileType
{
    enum Value
    {
        FILE,
        DIRECTORY
    };
};

/**
 * The ObjectDatabaseRecognizer class contains all information about
 */
class ObjectDatabaseRecognizer
{
    /**
    * database
    */
    database mDatabase;

    /**
    * indicates if read or not
    */
    bool mRead;

    /**
    * holds the entries of the recognizer.
    */
    ObjectDatabaseEntryPtrMap mEntries;
public:
    /**
    * Creates an empty object recognizer object.
    */
    ObjectDatabaseRecognizer(database db);

    /**
    * destructs the object
    */
    virtual ~ObjectDatabaseRecognizer()
    {}

    /**
    * @return the keyword associated with the recognizer
    */
    const std::string getKeyword();

    /**
    * @param kw the keyword for the recognizer.
    */
    void setKeyword(const std::string kw);

    /**
    * @return the path to the recognizer database folder.
    */
    const fs::path getPath();

    /**
    * @param pt the path to the recognizer database folder.
    */
    void setPath(const fs::path pt);

    /**
    * @return the regular expression for matching the recognizer's entry file or directory.
    */
    const std::string getRegexp();

    /**
    * @param the regular expression to use for matching the recognizer's entries.
    */
    void setRegexp(const std::string regexp);

    /**
    * @return if the entries of this recognizer are already read.
    */
    bool isRead();

private:
    /**
    * @param rd if the entries are read or not.
    */
    void setRead(bool rd);

public:
    /**
    * Reads the entries to
    */
    void readEntries();

    /**
    * @return the entries of the recognizer
    */
    ObjectDatabaseEntryPtrMap getEntries();

    /**
    * @param objectName the name of the object entry to be found
    * @return a special entry.
    */
    const ObjectDatabaseEntryPtr& getEntry(const std::string objectName);
};
}

#endif // OBJECT_DATABASE_CATEGORY_H_

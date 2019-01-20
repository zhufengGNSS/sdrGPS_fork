#ifndef CFX_CONFIGPARSER_H
#define CFX_CONFIGPARSER_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "softgps.h"

using namespace std;

/// A configuration file variable
/** Stores a variable from a configuration file. It is designed in a way
    that the variable 'is a' value to be represented in the configuration
    file (thus the overloaded conversion and assignment operators) while
    it 'has a' name (.getName() / .setName()).

    Blank lines will be stored as empty values (T_NONE) which don't have
    a name. This includes comments, which have no valuable meaning for
    the parser (any line containing beginning with '#', '//' or ';').
    All other types of variables simply store their name and the value as
    well as their native type in the configuration file.

    Note that booleans are not directly supported in the configuration
    file. These are implicitely converted from other types (any number
    unequal to zero and any string with a length > 0). When you save a
    boolean, it will be written as either '1' or '0'.
    Any other way of handling booleans would open the problem of 
    localization and the decision of which identifiers to recognize 
    ('true'/'false', 'on'/'off', 'yes'/'no', ...). If an application
    requires this, it should query the values in question as strings and
    check these for matches with the desired names.
*/
    enum TYPE {
      T_NONE = 0,                                     ///< No value / empty
      T_COMMENT,                                      ///< Comment
      T_BOOL,                                         ///< Boolean value
      T_LONG,                                         ///< Long value
      T_HEX,                                          ///< Hex value
      T_DOUBLE,                                       ///< Double value
      T_STRING                                        ///< String value
    };


class  CConfigVariable {
  public:
    /// Configuration variable types


    /// Constructor
    CConfigVariable(void);
    
    /// Constructor
    CConfigVariable(const std::string &sContents);
    /// Copy constructor
    CConfigVariable(const CConfigVariable &Variable);
    /// Destructor
    ~CConfigVariable(void);

  //
  // CConfigVariable implementation
  //
  public:
    /// Get variable name
    const std::string &getName(void) const;
    /// Set variable name
    void setName(const std::string &sName);

    /// Get variable type
    TYPE getType(void) const;

    /// Retrieve as boolean
    bool getBool(void) const;
    /// Retrieve as long
    long getLong(void) const;
    /// Retrieve as double
    double getDouble(void) const;
    /// Retrieve as string
    std::string getString(void) const;

    /// Set to boolean
    void setBool(bool bValue);
    /// Set to long
    void setLong(long nValue);
    /// Set to double
    void setDouble(double dValue);
    /// Set to string
    void setString(const std::string &sValue);

    /// Assign copy
    CConfigVariable &operator =(const CConfigVariable &Value);
    /// Assign boolean
    CConfigVariable &operator =(bool bValue) { setBool(bValue); return *this; }
    /// Assign integer (for convenience)
    CConfigVariable &operator =(int nValue) { setLong(nValue); return *this; }
    /// Assign long
    CConfigVariable &operator =(long nValue) { setLong(nValue); return *this; }
    /// Assign double
    CConfigVariable &operator =(double dValue) { setDouble(dValue); return *this; }
    /// Assign char array (for convenience)
    CConfigVariable &operator =(const char *pszValue) { setString(pszValue); return *this; }
    /// Assign string
    CConfigVariable &operator =(const std::string &sValue) { setString(sValue); return *this; }

    /// Convert to boolean
    inline operator bool(void) const { return getBool(); }
    /// Convert to long
    inline operator long(void) const { return getLong(); }
    /// Convert to double
    operator double(void) const { return getDouble(); }
    /// Convert to string
    operator std::string(void) const { return getString(); }

    /// Clear variable
    void clearContents(void);
    /// Get configuration contents
    const std::string &getContents(void) const;
    /// Set configuration contents
    std::string::size_type setContents(const std::string &sContents, std::string::size_type Start = 0);

  private:
    /// Clear the value
    void clearValue(void);
    /// Scan for the assigned value
    std::string::size_type scanValue(const std::string &sString, std::string::size_type Start);
    /// Scan a comment
    std::string::size_type scanComment(const std::string &sString, std::string::size_type Start);
    /// Scan a string assignment
    std::string::size_type scanString(const std::string &sString, std::string::size_type Start);
    /// Scan a number assignment
    std::string::size_type scanNumber(const std::string &sString, std::string::size_type Start);

    std::string *m_psName;                            ///< Variable name
    TYPE         m_eType;                             ///< Variable type
    union {
      bool         m_bValue;                          ///< Boolean value
      long         m_lValue;                          ///< Long/Hex value
      double       m_dValue;                          ///< Double value
      std::string *m_psValue;                         ///< String value
    };
    
    mutable bool                    m_bChanged;       ///< Has changed
    mutable std::string            *m_psContents;     ///< File contents
    mutable std::string::size_type  m_NameStart;      ///< Name starting offset
    mutable std::string::size_type  m_NameLength;     ///< Name length
    mutable std::string::size_type  m_ValueStart;     ///< Value starting offset
    mutable std::string::size_type  m_ValueLength;    ///< Value length
};







/// A configuration file section
/** Stores the contents of a section in the configuration file. Sections
    are declared in brackets (eg. '[Section]') and enable the grouping of
    values to further organize the configuration file.

    All variables declared in the configuration file before the first
    section begins are added to a nameless section.
*/
class  CConfigSection {
  public:
    /// Config variable enumerator
  //    class  CVariableEnumerator;

    /// Constructor
    CConfigSection(void);
    /// Constructor
    CConfigSection(const std::string &sContents);
    /// Copy constructor
    CConfigSection(const CConfigSection &Section);
    /// Destructor
    ~CConfigSection(void);

  //
  // CConfigSection implementation
  //
  public:
    /// Get section name
    const std::string &getName(void) const;
    /// Set section name
    void setName(const std::string &sName);

    /// Get number of variables in section
    unsigned long getNumVariables(void) const;

    /// Get variable
    CConfigVariable &getVariable(unsigned long lIndex);
    /// Get variable 
    CConfigVariable &getVariable(const std::string &sName);

    /// Add variable
    void addVariable(const CConfigVariable &Variable);
    /// Remove variable
    void removeVariable(unsigned long lIndex);
    /// Remove variable
    void removeVariable(const std::string &sName);

    /// Clear section
    void clearContents(void);
    /// Get configuration contents
    std::string getContents(void) const;
    /// Set configuration contents
    std::string::size_type setContents(const std::string &sContents, std::string::size_type Start = 0);

  private:
    /// A map of configuration variables
    typedef std::vector<CConfigVariable> VariableVector;

    std::string    *m_psName;                         ///< Section name
    VariableVector *m_pVariables;                     ///< Variables in section

    mutable bool                    m_bChanged;       ///< Has changed
    mutable std::string            *m_psContents;     ///< Section line
    mutable std::string::size_type  m_NameStart;      ///< Name starting offset
    mutable std::string::size_type  m_NameLength;     ///< Name length
};

class  CConfigFile {
  public:

    /// Constructor
    CConfigFile(void);
    /// Constructor
    CConfigFile(ifstream &);
    /// Constructor
    CConfigFile(const std::string &);
    /// Destructor
    ~CConfigFile(void);

  //
  // CConfigFile implementation
  //
  public:
    /// Get number of variables in file
    unsigned long getNumSections(void) const;

    /// Get variable
    CConfigSection &getSection(unsigned long lIndex);
    /// Get variable 
    CConfigSection &getSection(const std::string &sName);

    /// Add variable
    void addSection(const CConfigSection &Section);
    /// Remove variable
    void removeSection(unsigned long lIndex);
    /// Remove variable
    void removeSection(const std::string &sName);

   /// Clear section
   void clearContents(void);
   /// Get configuration contents
   std::string getContents(void) const;
   /// Set configuration contents
   std::string::size_type setContents(const std::string &sContents);

   void ReadConfigInfo( SOFTGPS_config  *);
   void PrintoutConfigInfo(const SOFTGPS_config&) ;
   
  private:
    /// A map of configuration variables
    typedef std::vector<CConfigSection> SectionVector;

    SectionVector *m_pSections;                       ///< Sections in file
};

#endif  //CFX_CONFIGPARSER_H

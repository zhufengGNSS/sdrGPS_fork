/*:configparser.cc
 **************************************************************
 * Configure file parser 
 *
 * This class is used to read and parse the configuration information from 
 * config file.
 *
 * Authors:
 *  Somebody else, got this file from internet
 *  Thanks to the original author
 *
 *Changed by: 
 *	Yu Lu, softwareGNSS@gmail.com
 *      Feb, 2005
 *
 **************************************************************/

#include "includes/configparser.h"
#include <stdio.h> 
#include <stdlib.h>
#include <stdexcept>
#include <sstream>
#include <string.h>


/// Convert integer to string
static std::string itos(long nValue);
/// Convert integer to string
static std::string htos(long nValue);
/// Convert double to string
static std::string ftos(double dValue, long nDecimals = 6);

/// Check whether the character is a letter
/** Returns true if the specified character is a letter, being either
    in range from a-z (upper or lower case) or the special character '_'

    @param  Char  Character to check
    @return True if the character is a letter
*/
static inline bool isletter(std::string::value_type Char) {
  return (((Char >= 'a') && (Char <= 'z')) || ((Char >= 'A') && (Char <= 'Z')) || (Char == '_'));
}

/// Check whether the character is numeric
/** Returns true if the specified character is a number, being
    in the range from 0-9.

    @param  Char  Character to check
    @return True if the character is a number
*/
static inline bool isnumeric(std::string::value_type Char) {
  return (((Char >= '1') && (Char <= '9')) || (Char == '0'));
}

/// Check whether the character is a hexadecimal number
/** Returns true if the specified character is a hexadecimal number,
    being in the range from 0-9 or a-z (upper or lower case)

    @param  Char  Character to check
    @return True if the character is a hexadecimal number
*/
static inline bool ishex(std::string::value_type Char) {
  return (((Char >= '1') && (Char <= '9')) || ((Char >= 'a') && (Char <= 'f')) || ((Char >= 'A') && (Char <= 'F'))  || (Char == '0'));
}

// ####################################################################### //
// # CFX::CConfigVariable::CConfigVariable()                 Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigVariable
*/
CConfigVariable::CConfigVariable(void) :
  m_psName(new std::string()),
  m_eType(T_NONE),
  m_bChanged(false),
  m_psContents(NULL) {
  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::CConfigVariable()                 Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigVariable

    @param  sContents  Initial contents string
*/
CConfigVariable::CConfigVariable(const std::string &sContents) :
  m_psName(new std::string()),
  m_eType(T_NONE),
  m_bChanged(false),
  m_psContents(NULL) {
  if(sContents[sContents.length() - 1] != '\n')
    setContents(sContents + '\n');
  else
    setContents(sContents);

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::CConfigVariable()                 Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigVariable as copy of an
    existing instance
    
    @param  Variable  Variable to copy
*/
CConfigVariable::CConfigVariable(const CConfigVariable &Variable) :
  m_psName(new std::string(*Variable.m_psName)),
  m_eType(Variable.m_eType),
  m_bChanged(false),
  m_psContents(NULL),
  m_NameStart(Variable.m_NameStart),
  m_NameLength(Variable.m_NameLength),
  m_ValueStart(Variable.m_ValueStart),
  m_ValueLength(Variable.m_ValueLength) {

  if((m_eType == T_STRING) || (m_eType == T_STRING))
    m_psValue = new std::string(*Variable.m_psValue);

  if(Variable.m_psContents)
    m_psContents = new std::string(*Variable.m_psContents);

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::~CConfigVariable()                 Destructor # //
// ####################################################################### //
/** Destroys an instance of CConfigVariable
*/
CConfigVariable::~CConfigVariable(void) {
  clearContents();
  delete m_psName;

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::clearContents()                               # //
// ####################################################################### //
/** Clears the variable's contents
*/
void CConfigVariable::clearContents(void) {
  clearValue();  

  delete m_psContents;
  m_psContents = NULL;
  *m_psName = "";

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::clearValue()                                  # //
// ####################################################################### //
/** Clears the variable's contents
*/
void CConfigVariable::clearValue(void) {
  if(m_eType == T_STRING)
    delete m_psValue;
  
  m_eType = T_NONE;

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::getContents()                                 # //
// ####################################################################### //
/** Retrieves the configuration file contents for this variable

    @return The configuration file contents for the variable
*/
const std::string &CConfigVariable::getContents(void) const { 
  static std::string sEmpty;

  // If the variable hasn't changed, just return its current contents
  if(!m_bChanged) {
    if(m_psContents)
      return *m_psContents;
    else
      return sEmpty;

  // The variable has been changed since last time
  } else {

    // Has the contents string been created already ?
    if(m_psContents) {
      switch(m_eType) {
        case T_BOOL:
        case T_LONG:
        case T_HEX:
        case T_DOUBLE:
        case T_STRING: {
            *m_psContents = m_psContents->substr(0, m_NameStart) + *m_psName + m_psContents->substr(m_NameStart + m_NameLength);
            m_ValueStart += m_psName->length() - m_NameLength;
            m_NameLength = m_psName->length();

          // Place quotation marks around strings
          if(m_eType == T_STRING) {
            *m_psContents = m_psContents->substr(0, m_ValueStart) + '"' + getString() + '"' + m_psContents->substr(m_ValueStart + m_ValueLength);
            m_ValueLength = getString().length() + 2;
          } else {
            *m_psContents = m_psContents->substr(0, m_ValueStart) + getString() + m_psContents->substr(m_ValueStart + m_ValueLength);
            m_ValueLength = getString().length();
          }

          m_bChanged = false;
          return *m_psContents;
        }

        default: {
          delete m_psContents;
          m_psContents = NULL;
          break;
        }
      }

      return sEmpty;
 
    // The contents string wasn't created yet
    } else {
      switch(m_eType) {
        case T_BOOL:
        case T_LONG:
        case T_HEX:
        case T_DOUBLE:
        case T_STRING: {
          // Place quotation marks around strings
          if(m_eType == T_STRING) {
            m_psContents = new std::string(*m_psName + " = \"" + getString() + "\"\n");
            m_NameStart = 0;
            m_NameLength = m_psName->length();
            m_ValueStart = m_NameLength + 3;
            m_ValueLength = getString().length() + 2;
          } else {
            m_psContents = new std::string(*m_psName + " = " + getString() + "\n");
            m_NameStart = 0;
            m_NameLength = m_psName->length();
            m_ValueStart = m_NameLength + 3;
            m_ValueLength = getString().length();
          }

          m_bChanged = false;
          return *m_psContents;
        }

        default: {
          delete m_psContents;
          m_psContents = NULL;
          break;
        }
      }

      return sEmpty;
    }
  }
}

// ####################################################################### //
// # CFX::CConfigVariable::setContents()                                 # //
// ####################################################################### //
/** Sets the configuration file contents for this variable

    @param  sContents  The configuration file contents for the variable
*/
std::string::size_type CConfigVariable::setContents(const std::string &sContents,
                                                    std::string::size_type Start) {
  std::string::size_type Pos = Start;

  clearContents();

  // Ensure that the string is not empty
  if(sContents.length() != 0) {

    // Look for the first non-spacing character
    Pos = sContents.find_first_not_of(" \t", Pos);
    if(Pos != std::string::npos) {
      std::string::size_type LastNonSpacePos = std::string::npos;
      m_NameStart = Pos;

      // Scan the string for an assignment
      while(Pos < sContents.length()) {
        std::string::value_type Char = sContents[Pos];

        if(Char == '\n')
          break;
        else if((Char == '\r') && (Char == '\n'))
          break;

        // If a comment is initiated, stop parsing
        if((Char == '#') || (Char == ';')) {
          Pos = scanComment(sContents, Pos);
          break;
        }
        if((Char == '/') && (sContents[Pos + 1] == '/')) {
          Pos = scanComment(sContents, Pos);
          break;
        }

        // If an assignment was found, try to retrieve the variable
        if(Char == '=') {
          if(LastNonSpacePos != std::string::npos) {
            m_NameLength = LastNonSpacePos - m_NameStart + 1;
            *m_psName = sContents.substr(m_NameStart, m_NameLength);

            Pos = scanValue(sContents, Pos + 1);
          }

          break;
        }

        // Update position of the last non-spacing character encountered
        if((Char != ' ') && (Char != '\t'))
          LastNonSpacePos = Pos;

        Pos++;
      }
    }

    std::string::size_type EndPos = sContents.find_first_of('\n', Pos);
    if(EndPos == std::string::npos)
      Pos = sContents.length();
    else
      Pos = EndPos + 1;
  }

  m_NameStart -= Start;
  m_ValueStart -= Start;
  m_psContents = new std::string(sContents.substr(Start, Pos - Start));
  m_bChanged = false;

  return Pos;
}

// ####################################################################### //
// # CFX::CConfigVariable::scanValue()                                   # //
// ####################################################################### //
/** Scans the specified string for a value after an assignment

    @param  sString  String to scan
*/
std::string::size_type CConfigVariable::scanValue(const std::string &sString, std::string::size_type Start) {
  std::string::size_type Pos = Start;

  // Ensure that the string is not empty
  if(sString.length() > 0) {

    // Look for the first non-spacing character
    Pos = sString.find_first_not_of(" \t", Start);
    if(Pos != std::string::npos) {
      //std::string::value_type Char = sString[Pos];

      // Try as number and fallback to string on invalid characters
      std::string::size_type Length = scanNumber(sString, Pos);
      if(Length == std::string::npos)
        Pos = scanString(sString, Pos);
      else
        Pos = Length;
    }
  }

  return Pos;
}

// ####################################################################### //
// # CFX::CConfigVariable::scanComment()                                 # //
// ####################################################################### //
/** Scans the specified string for a string assignment

    @param  sString  String to scan
*/
std::string::size_type CConfigVariable::scanComment(const std::string &sString, std::string::size_type Pos) {
  if(sString.length() > 0) {
    while(Pos < sString.length()) {
      if(sString[Pos] == '\n')
        break;

      Pos++;
    }
  }

  return Pos;
}

// ####################################################################### //
// # CFX::CConfigVariable::scanString()                                  # //
// ####################################################################### //
/** Scans the specified string for a string assignment

    @param  sString  String to scan
*/
std::string::size_type CConfigVariable::scanString(const std::string &sString, std::string::size_type Start) {
  std::string::size_type Pos = Start;

  // Ensure that the string is not empty
  if(sString.length() > 0) {

    // Look for the first non-spacing character
    Pos = sString.find_first_not_of(" \t", Pos);
    if(Pos != std::string::npos) {
      std::string::value_type Char = sString[Pos];

      // If it starts with a quote, scan it using extended mode
      if((Char == '"') || (Char == '\n') || 
         ((Char == '\r') && (sString[Pos + 1] == '\n'))) {
        while(Pos < sString.length()) {
          Char = sString[Pos];

          if((Char == '\r') && (sString[Pos + 1] == '\n')) {
            Pos++;
            Char = sString[Pos];
          }
          
          if(Char == '"')
            break;
          else if((Char != ' ') && (Char != '\t') && (Char != '\n'))
            return Pos;
          
          Pos++;
        }

        std::string sResult;

        m_ValueStart = Pos;
        std::string::size_type LastStringPos = Pos;

        Pos++;

        // Scan the string until a final closing quote is found
        while(Pos < sString.length()) {
          std::string::value_type Char = sString[Pos];

          // Found closing quote ?
          if(Char == '"') {
            bool bContinue = false;

            // Check if the string is reopened with another quote
            Pos++;
            LastStringPos = Pos;
            while(Pos < sString.length()) {
              Char = sString[Pos];
              Pos++;

              // New quote found, continue scanning at this location
              if(Char == '"') {
                bContinue = true;
                break;

              // Break on anything but spaces and newlines
              } else if((Char != ' ') && (Char != '\t') && (Char != '\n')) {
                if((Char != '\r') || (sString[Pos] != '\n'))
                  break;
              }/* else if(Char == '\n') {
                LastStringPos = Pos - 1;
              }*/
            }

            // Continue only if the string was reopened
            if(bContinue)
              continue;
            else
              break;
        
          // A character other than a quote was found
          } else {

            // Is this an escape sequence ?
            if(Char == '\\') {
              bool bValid = true;

              // Try to resolve the escape character
              switch(sString[Pos + 1]) {
                case 'n': Char = '\n'; break;
                case 't': Char = '\t'; break;
                case 'v': Char = '\v'; break;
                case 'b': Char = '\b'; break;
                case 'r': Char = '\r'; break;
                case 'f': Char = '\f'; break;
                case 'a': Char = '\a'; break;
                case '\\': Char = '\\'; break;
                case '?': Char = '?'; break;
                case '\'': Char = '\''; break;
                case '\"': Char = '\"'; break;
                case '0': Char = '\0'; break;
                case '\n': break;
                default: bValid = false; break;
              }

              // Skip the escape character only if the sequence was valid
              if(bValid)
                Pos++;
            }

            // Append the character to the result
            sResult += Char;
            LastStringPos = Pos;
          }

          Pos++;
        }

        m_eType = T_STRING;
        m_ValueLength = LastStringPos - m_ValueStart;
        m_psValue = new std::string(sResult);
        Pos = LastStringPos;

      // Scan a simple string
      } else {
        std::string::size_type LastNonSpacePos = Pos;
        m_ValueStart = Pos;

        while(Pos < sString.length()) {
          std::string::value_type Char = sString[Pos];

          // If a comment is initiated, stop parsing
          if((Char == '#') || (Char == ';'))
            break;
          if((Char == '/') && (sString[Pos + 1] == '/'))
            break;

          // Simple strings end with newline characters
          if((Char == '\n') || ((Char == '\r') && (sString[Pos + 1] == '\n')))
            break;

          if((Char != ' ') && (Char != '\t'))
            LastNonSpacePos = Pos;

          Pos++;
        }

        m_eType = T_STRING;
        m_ValueLength = LastNonSpacePos - m_ValueStart + 1;
        m_psValue = new std::string(sString.substr(m_ValueStart, m_ValueLength));
      }
    }
  }

  return Pos;
}

// ####################################################################### //
// # CFX::CConfigVariable::scanNumber()                                  # //
// ####################################################################### //
/** Scans the specified string for a number assignment

    @param  sString  String to scan
*/
std::string::size_type CConfigVariable::scanNumber(const std::string &sString, std::string::size_type Pos) {
  // Ensure that the string is not empty
  if(sString.length() > 0) {
    std::string::value_type Char = sString[Pos];
    bool bNegative = false;

    m_ValueStart = Pos;

    if((Char == '-') || (Char == '+')) {
      if(Char == '-')
        bNegative = true;

      Pos++;
      Char = sString[Pos];
    }

    // If it starts with a single quote, scan it as character constant
    if(Char == '\'') {
      Pos++;

      // Build the number by shifting and oring the current character
      long lValue = 0;
      while(Pos < sString.length()) {
        Char = sString[Pos];

        // Break if a closing single quote is found
        if(Char == '\'') {
          break;

        } else {
          // Is this an escape sequence ?
          if(Char == '\\') {
            bool bValid = true;

            // Try to resolve the escape character
            switch(sString[Pos + 1]) {
              case 'n': Char = '\n'; break;
              case 't': Char = '\t'; break;
              case 'v': Char = '\v'; break;
              case 'b': Char = '\b'; break;
              case 'r': Char = '\r'; break;
              case 'f': Char = '\f'; break;
              case 'a': Char = '\a'; break;
              case '\\': Char = '\\'; break;
              case '?': Char = '?'; break;
              case '\'': Char = '\''; break;
              case '\"': Char = '\"'; break;
              case '0': Char = '\0'; break;
              case '\n': break;
              default: bValid = false; break;
            }

            // Skip the escape character only if the sequence was valid
            if(bValid)
              Pos++;
          }

          lValue <<= (sizeof(std::string::value_type) * 8);
          lValue |= Char;
        }

        Pos++;
      }
      if(Pos < sString.length()) {
        Char = sString[Pos + 1];
        if((Char != ' ') && (Char != '\t') && (Char != '\n'))
          if((Char != '\r') || (sString[Pos + 1] != '\n'))
            return std::string::npos;
      }

      if(bNegative)
        lValue = -lValue;

      m_eType = T_LONG;
      m_ValueLength = Pos - m_ValueStart;
      m_lValue = lValue;
    
    // No character constant initiated
    } else {
      long lValue = 0;

      // If it starts with 0x, interpret it as hexadecimal number
      if((Char == '0') && ((sString[Pos + 1] == 'x') || (sString[Pos + 1] == 'X'))) {
        Pos += 2;

        // Scan the string and build its hexadecimal value
        while(Pos < sString.length()) {
          Char = sString[Pos];

          bool bValid = true;
          switch(Char) {
            case '0': Char = 0; break;
            case '1': Char = 1; break;
            case '2': Char = 2; break;
            case '3': Char = 3; break;
            case '4': Char = 4; break;
            case '5': Char = 5; break;
            case '6': Char = 6; break;
            case '7': Char = 7; break;
            case '8': Char = 8; break;
            case '9': Char = 9; break;
            case 'a': case 'A': Char = 10; break;
            case 'b': case 'B': Char = 11; break;
            case 'c': case 'C': Char = 12; break;
            case 'd': case 'D': Char = 13; break;
            case 'e': case 'E': Char = 14; break;
            case 'f': case 'F': Char = 15; break;
            default: bValid = false; break;
          }
          if(bValid) {
            lValue <<= 4;
            lValue |= Char;
          } else {
            return std::string::npos;
          }

          Pos++;
        }

        m_eType = T_HEX;
        m_ValueLength = Pos - m_ValueStart;
        m_lValue = lValue;

      // Read the number as integer or floating point value
      } else {
        std::string::size_type Start = Pos;

        bool bDecimals = false;
        while(Pos < sString.length()) {
          Char = sString[Pos];

          if(Char == '.') {
            if(bDecimals)
              return std::string::npos;
              
            bDecimals = true;
          } else if(!isnumeric(Char)) {
            return std::string::npos;
          }

          Pos++;
        }

        // Convert the number either as integer or as floating point value
        if(bDecimals) {
          m_eType = T_DOUBLE;
          m_dValue = ::atof(((bNegative ? std::string("-") : std::string()) + sString.substr(Start, Pos - Start)).c_str());
        } else {
          m_eType = T_LONG;
          m_lValue = ::atoi(((bNegative ? std::string("-") : std::string()) + sString.substr(Start, Pos - Start)).c_str());
        }
        m_ValueLength = Pos - m_ValueStart;
      }
    }
  }

  return Pos;
}

// ####################################################################### //
// # CFX::CConfigVariable::getName()                                     # //
// ####################################################################### //
/** Retrieves the name of the configuration variable

    @return The variable's name
*/
const std::string &CConfigVariable::getName(void) const {
  return *m_psName;
}

// ####################################################################### //
// # CFX::CConfigVariable::setName()                                     # //
// ####################################################################### //
/** Sets the name of the configuration variable

    @param  sName  New name for the variable
*/
void CConfigVariable::setName(const std::string &sName) {
  *m_psName = sName;
  m_bChanged = true;

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::getType()                                     # //
// ####################################################################### //
/** Retrieves the type of the configuration variable

    @return The variable's type
*/
TYPE CConfigVariable::getType(void) const {
  return m_eType;
}

// ####################################################################### //
// # CFX::CConfigVariable::getBool()                                     # //
// ####################################################################### //
/** Retrieves the configuration value as boolean

    @return The variable's boolean representation
*/
bool CConfigVariable::getBool(void) const {
  switch(m_eType) {
    case T_BOOL: return m_bValue;
    case T_LONG: return m_lValue != 0;
    case T_HEX: return m_lValue != 0;
    case T_DOUBLE: return m_dValue != 0.0;
    case T_STRING: return m_psValue->length() > 0;
    default: return false;
  }
}

// ####################################################################### //
// # CFX::CConfigVariable::getLong()                                     # //
// ####################################################################### //
/** Retrieves the configuration value as long

    @return The variable's long representation
*/
long CConfigVariable::getLong(void) const {
  switch(m_eType) {
    case T_BOOL: return static_cast<long>(m_bValue);
    case T_LONG: return m_lValue;
    case T_HEX: return m_lValue;
    case T_DOUBLE: return static_cast<long>(m_dValue + 0.5);
    case T_STRING: return ::atoi(m_psValue->c_str());
    default: return 0;
  }
}

// ####################################################################### //
// # CFX::CConfigVariable::getDouble()                                   # //
// ####################################################################### //
/** Retrieves the configuration value as double

    @return The variable's double representation
*/
double CConfigVariable::getDouble(void) const {
  switch(m_eType) {
    case T_BOOL: return static_cast<double>(m_bValue);
    case T_LONG: return static_cast<double>(m_lValue);
    case T_HEX: return static_cast<double>(m_lValue);
    case T_DOUBLE: return m_dValue;
    case T_STRING: return ::atof(m_psValue->c_str());
    default: return 0.0;
  }
}

// ####################################################################### //
// # CFX::CConfigVariable::getString()                                   # //
// ####################################################################### //
/** Retrieves the configuration value as string

    @return The variable's string representation
*/
std::string CConfigVariable::getString(void) const {
  switch(m_eType) {
    case T_BOOL: return m_bValue ? "1" : "0";
    case T_LONG: return itos(m_lValue);
    case T_HEX: return htos(m_lValue);
    case T_DOUBLE: return ftos(m_dValue);
    case T_STRING: return *m_psValue;
    default: return "";
  }
}

// ####################################################################### //
// # CFX::CConfigVariable::operator =()                                  # // 
// ####################################################################### //
/** Assigns a value to the CConfigVariable. If the assigned value is of type
    Blob or Object, AddRef() will automatically be called on it.

    @param  Value  Value to assign
    @return The assigned value
*/
CConfigVariable &CConfigVariable::operator =(const CConfigVariable &Value) {
  clearContents();

  m_eType = Value.m_eType;
  switch(m_eType) {
   case T_NONE:
    case T_BOOL: m_bValue = Value.m_bValue; break;
    case T_LONG: case T_HEX: m_lValue = Value.m_lValue; break;
    case T_DOUBLE: m_dValue = Value.m_dValue; break;
    case T_STRING: case T_COMMENT: m_psValue = new std::string(*Value.m_psValue); break;
  }
  m_bChanged = true;

  return *this;
}

// ####################################################################### //
// # CFX::CConfigVariable::setBool()                                     # // 
// ####################################################################### //
/** Assigns a boolean value to the CConfigVariable

    @param  bValue  Boolean value
    @return The assigned value
*/
void CConfigVariable::setBool(bool bValue) {
  clearValue();

  m_eType = T_BOOL;
  m_bValue = bValue;
  m_bChanged = true;

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::setLong()                                     # // 
// ####################################################################### //
/** Assigns an integer value to the CConfigVariable

    @param  nValue  Integer value
    @return The assigned value
*/
void CConfigVariable::setLong(long nValue) {
  clearValue();

  m_eType = T_LONG;
  m_lValue = nValue;
  m_bChanged = true;

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::setDouble()                                   # // 
// ####################################################################### //
/** Assigns a double value to the CConfigVariable

    @param  dValue  Double value
    @return The assigned value
*/
void CConfigVariable::setDouble(double dValue) {
  clearValue();

  m_eType = T_DOUBLE;
  m_dValue = dValue;
  m_bChanged = true;

  return;
}

// ####################################################################### //
// # CFX::CConfigVariable::setString()                                   # // 
// ####################################################################### //
/** Assigns a string value to the CConfigVariable

    @param  sString  String value
    @return The assigned value
*/
void CConfigVariable::setString(const std::string &sValue) {
  clearValue();

  m_eType = T_STRING;
  m_psValue = new std::string(sValue);
  m_bChanged = true;

  return;
}


// ####################################################################### //
// # my_itoa()
// ####################################################################### //

std::string itoa( int value, char* dest, unsigned int base)
{
  const char digitMap[] = "0123456789abcdef";
  std::string buf;

  // guard

  if( base==0 || base>16 )
    // Error : may add more trace/log output here
    return buf;

  // Take care negative int
  std::string sign;
  int _value = value;
  if( value<0)
    {
      _value = -value;
      sign="-";
    }
  
  // Translating number to string with base:
  for( int i=30; _value && i; --i)
    {
      buf = digitMap[ _value%base ] + buf;
      _value /= base;
    }
  
  sign.append(buf);

  strcpy( dest, sign.c_str());
  return sign;
}



// ####################################################################### //
// # itos()                                                              # // 
// ####################################################################### //
/** Converts an integer into a string

    @param  nValue  Integer value to convert
    @return The string representation of the integer
*/
std::string itos(long nValue) {
  char pszValue[11];
  itoa(nValue, pszValue, 10);

  return pszValue; // Implicitely constructs an std::string
}



// ####################################################################### //
// # htos()                                                              # // 
// ####################################################################### //
/** Converts an integer into a string

    @param  nValue  Integer value to convert
    @return The string representation of the integer
*/
std::string htos(long nValue) {
  char pszValue[11];
  pszValue[0] = '0';
  pszValue[1] = 'x';
  itoa(nValue, &pszValue[2], 16);

  return pszValue; // Implicitely constructs an std::string
}

// ####################################################################### //
// # ftos()                                                              # // 
// ####################################################################### //
/** Converts a double into a string

    @param  dValue     Double value to convert
    @param  nDecimals  Number of decimal places to return
    @return The string representation of the double
*/
std::string ftos(double dValue, long nDecimals) {

   char tmp_str[80];
   
   //tmp_str = fcvt(dValue, (int)nDecimals, &nDec, &nSign);
   std::string sTemp;// = ::fcvt(dValue, nDecimals, &nDec, &nSign);
   sprintf(tmp_str, "%E", dValue);
   sTemp = tmp_str;
   return sTemp;
   /*
  return (((nDec == 0) ? (nSign ? std::string("-0") : std::string("0"))
                       : (nSign ? std::string("-") : std::string())) +
          sTemp.substr(0, nDec) + "." + sTemp.substr(nDec)).c_str();
    */ 
}

// ####################################################################### //
// # CFX::CConfigSection::CConfigSection()                   Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigSection
*/
CConfigSection::CConfigSection(void) :
  m_psName(new std::string()),
  m_pVariables(new VariableVector()),
  m_bChanged(false),
  m_psContents(NULL) {
  return;
}

// ####################################################################### //
// # CFX::CConfigSection::CConfigSection()                   Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigSection

    @param  sContents  Initial contents string
*/
CConfigSection::CConfigSection(const std::string &sContents) :
  m_psName(new std::string()),
  m_pVariables(new VariableVector()),
  m_bChanged(false),
  m_psContents(NULL) {

  if(sContents[sContents.length() - 1] != '\n')
    setContents(sContents + '\n');
  else
    setContents(sContents);

  return;
}


// ####################################################################### //
// # CFX::CConfigSection::CConfigSection()                   Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigSection as copy of an
    existing instance
    
    @param  Section  Section to copy
*/
CConfigSection::CConfigSection(const CConfigSection &Section) :
  m_psName(new std::string(*Section.m_psName)),
  m_pVariables(new VariableVector(*Section.m_pVariables)),
  m_bChanged(Section.m_bChanged),
  m_psContents(NULL),
  m_NameStart(Section.m_NameStart),
  m_NameLength(Section.m_NameLength) {

  if(Section.m_psContents)
    m_psContents = new std::string(*Section.m_psContents);

  return;
}

// ####################################################################### //
// # CFX::CConfigSection::~CConfigSection()                   Destructor # //
// ####################################################################### //
/** Destroys an instance of CConfigSection
*/
CConfigSection::~CConfigSection(void) {
  clearContents();
  delete m_pVariables;
  delete m_psName;

  return;
}

// ####################################################################### //
// # CFX::CConfigSection::clearContents()                                # //
// ####################################################################### //
/** Clears the section's contents
*/
void CConfigSection::clearContents(void) {
  delete m_psContents;
  m_psContents = NULL;

  *m_psName = "";
  m_pVariables->clear();

  return;
}

// ####################################################################### //
// # CFX::CConfigSection::getContents()                                  # //
// ####################################################################### //
/** Retrieves the configuration file contents for this variable

    @return The configuration file contents for the variable
*/
std::string CConfigSection::getContents(void) const { 
  std::string sContents;

  if(!m_bChanged) {
    if(m_psContents)
      sContents = *m_psContents;
    else
      sContents = "";
  } else {
    if(m_psContents) {
      *m_psContents = m_psContents->substr(0, m_NameStart) + *m_psName + m_psContents->substr(m_NameStart + m_NameLength);
      m_NameLength = m_psName->length();
      m_bChanged = false;

      sContents = *m_psContents;
    } else {
      m_psContents = new std::string(std::string("[") + *m_psName + "]\n");
      m_NameStart = 1;
      m_NameLength = m_psName->length();
      m_bChanged = false;

      sContents = *m_psContents;
    }
  }

  for(unsigned long lVariable = 0; lVariable < m_pVariables->size(); lVariable++)
    sContents += m_pVariables->operator[](lVariable).getContents();

  return sContents;
}

// ####################################################################### //
// # CFX::CConfigSection::setContents()                                  # //
// ####################################################################### //
/** Sets the configuration file contents for this variable

    @param  sContents  The configuration file contents for the variable
*/
std::string::size_type CConfigSection::setContents(const std::string &sContents, std::string::size_type Start) {
  std::string::size_type Pos = Start;

  clearContents();

  // Ensure that the string is not empty
  if(sContents.length() != 0) {
    // Look for a section declaration
    Pos = sContents.find_first_not_of(" \t", Start);
    if((Pos != std::string::npos) && (sContents[Pos] == '[')) {
      Pos++;

      // Look for the first non-spacing character in the section
      while(Pos < sContents.length()) {
        std::string::value_type Char = sContents[Pos];

        if((Char != ' ') && (Char != '\t'))
          break;

        Pos++;
      }
      std::string::size_type LastNonSpacePos = Pos;

      // Scan the remaining string for a closing bracket
      m_NameStart = Pos;
      while(Pos < sContents.length()) {
        std::string::value_type Char = sContents[Pos];
        
        if((Char == ']') || (Char == '\n'))
          break;
        
        if(Char == '\r') {
          if(sContents[Pos + 1] == '\n') {
            Pos++;
            break;
          }
        } else if((Char != ' ') && (Char != '\t')) {
          LastNonSpacePos = Pos + 1;
        }

        Pos++;
      }
      Pos++;

      m_NameLength = LastNonSpacePos - m_NameStart;

      // The section name was extracted successfully
      *m_psName = sContents.substr(m_NameStart, m_NameLength);

      Pos = sContents.find_first_of('\n', Pos) + 1;
      m_NameStart -= Start;
      m_psContents = new std::string(sContents.substr(Start, Pos - Start));

    } else {
      Pos = Start;
      m_psContents = NULL; //new std::string();
    }

    // Parse all variables in this section
    while(Pos < sContents.length()) { 
      std::string::value_type Char=0;

      std::string::size_type Bracket = Pos;
      while(Bracket < sContents.length()) {
        Char = sContents[Bracket];

        if((Char != ' ') && (Char != '\t'))
          break;

        Bracket++;
      }

      // Stop parsing when a new section begins
      if(Char == '[') {
        break;

      } else {
        CConfigVariable Variable;

        Pos = Variable.setContents(sContents, Pos);

	//	cout << Variable.getName()<< endl;

	if( Variable.getName() != "" ) // only when the name is not blank
	  {
	    if(m_pVariables->size() >= m_pVariables->capacity())
	      m_pVariables->reserve(m_pVariables->size() + 16);
	    
	    m_pVariables->push_back(Variable);
	  }
      }
    }
    m_pVariables->reserve(m_pVariables->size());
    
  } else {
    m_psContents = new std::string();
  }
  
  return Pos;
}

// ####################################################################### //
// # CFX::CConfigSection::getName()                                      # //
// ####################################################################### //
/** Retrieves the name of the configuration section

    @return The section's name
*/
const std::string &CConfigSection::getName(void) const {
  return *m_psName;
}

// ####################################################################### //
// # CFX::CConfigSection::setName()                                      # //
// ####################################################################### //
/** Sets the name of the configuration section

    @param  sName  New name for the section
*/
void CConfigSection::setName(const std::string &sName) {
  *m_psName = sName;
  m_bChanged = true;
  return;
}

// ####################################################################### //
// # CFX::CConfigSection::getNumVariables()                              # //
// ####################################################################### //
/** Get number of variables in the config section

    @return The total number of variables
*/
unsigned long CConfigSection::getNumVariables(void) const {
  return m_pVariables->size();
}

// ####################################################################### //
// # CFX::CConfigSection::getVariable()                                  # //
// ####################################################################### //
/** Retrieve a variable from the section

    @param  lIndex  Index of the variable to retrieve
    @return The variable's index
*/
CConfigVariable &CConfigSection::getVariable(unsigned long lIndex) {
  if(lIndex >= m_pVariables->size())
    throw std::out_of_range("The specified index is out of range");

  return m_pVariables->operator[](lIndex);
}

// ####################################################################### //
// # CFX::CConfigSection::getVariable()                                  # //
// ####################################################################### //
/** Retrieve a variable from the section

    @param  sName  Name of the variable to retrieve
    @return The variable's index
*/
CConfigVariable &CConfigSection::getVariable(const std::string &sName) {
  VariableVector::iterator VariableEnd = m_pVariables->end();
  for(VariableVector::iterator VariableIt = m_pVariables->begin();
      VariableIt != VariableEnd;
      VariableIt++) {
    if(VariableIt->getName() == sName)
      return *VariableIt;
  }
  
  throw std::invalid_argument("A variable with the specified name could not be found");
}

// ####################################################################### //
// # CFX::CConfigSection::addVariable()                                  # //
// ####################################################################### //
/** Add a variable to the section

    @param  pVariable  Variable to add
    @return The variable's index
*/
void CConfigSection::addVariable(const CConfigVariable &Variable) {
  m_pVariables->push_back(Variable);

  return;
}

// ####################################################################### //
// # CFX::CConfigSection::removeVariable()                               # //
// ####################################################################### //
/** Remove a variable from the section

    @param  lIndex  Index of the variable to remove
*/
void CConfigSection::removeVariable(unsigned long lIndex) {
  if(lIndex >= m_pVariables->size())
    throw std::out_of_range("The specified index is out of range");

  m_pVariables->erase(m_pVariables->begin() + lIndex);

  return;
}

// ####################################################################### //
// # CFX::CConfigSection::removeVariable()                               # //
// ####################################################################### //
/** Remove a variable from the section

    @param  sName  Name of the variable to remove
*/
void CConfigSection::removeVariable(const std::string &sName) {
  bool bFound = false;

  VariableVector::iterator VariableEnd = m_pVariables->end();
  for(VariableVector::iterator VariableIt = m_pVariables->begin();
      VariableIt != VariableEnd;
      VariableIt++) {
    if(VariableIt->getName() == sName) {
      m_pVariables->erase(VariableIt);
      bFound = true;
    }
  }

  if(!bFound)
    throw std::invalid_argument("A variable with the specified name could not be found");
}

// ####################################################################### //
// # CFX::CConfigFile::CConfigFile()                         Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigFile
*/
CConfigFile::CConfigFile(void) :
  m_pSections(new SectionVector()) {

  return;
}

// ####################################################################### //
// # CFX::CConfigFile::CConfigFile()                         Constructor # //
// ####################################################################### //
/** Initializes an instance of CConfigFile
*/
CConfigFile::CConfigFile(const std::string &sContents) :
  m_pSections(new SectionVector()) {
  setContents(sContents);

  return;
}

// ####################################################################### //
// # CConfigFile::CConfigFile()
// ####################################################################### //
/** Initializes an instance of CConfigFile using ifstream object
 */
CConfigFile::CConfigFile( ifstream &cfgFile ) :
  m_pSections(new SectionVector())
{
  string psz;
  char ch;
  ostringstream buf;

  // first get the whole config info into buffer
  while( buf && cfgFile.get(ch) )
    buf.put(ch);

  psz = buf.str();

  setContents( psz );

}


// ####################################################################### //
// # CFX::CConfigFile::~CConfigFile()                         Destructor # //
// ####################################################################### //
/** Destroys an instance of CConfigFile
*/
CConfigFile::~CConfigFile(void) {
  clearContents();
  delete m_pSections;

  return;
}

// ####################################################################### //
// # CFX::CConfigFile::clearContents()                                   # //
// ####################################################################### //
/** Clears the file's contents
*/
void CConfigFile::clearContents(void) {
  m_pSections->clear();

  return;
}

// ####################################################################### //
// # CFX::CConfigFile::getContents()                                     # //
// ####################################################################### //
/** Retrieves the configuration file contents for this variable

    @return The configuration file contents for the variable
*/
std::string CConfigFile::getContents(void) const { 
  std::string sContents;

  for(unsigned long lSection = 0; lSection < m_pSections->size(); lSection++)
    sContents += m_pSections->operator[](lSection).getContents();

  return sContents;
}

// ####################################################################### //
// # CFX::CConfigFile::setContents()                                     # //
// ####################################################################### //
/** Sets the configuration file contents for this variable

    @param  sContents  The configuration file contents for the variable
*/
std::string::size_type CConfigFile::setContents(const std::string &sContents) {
  std::string::size_type Pos = 0;

  clearContents();

  // Ensure that the string is not empty
  if(sContents.length() != 0) {
    while(Pos < sContents.length()) {
      CConfigSection Section;

      Pos = Section.setContents(sContents, Pos);

      if(m_pSections->size() >= m_pSections->capacity())
        m_pSections->reserve(m_pSections->size() + 16);

      m_pSections->push_back(Section);
    }
  }

  return Pos;
}

// ####################################################################### //
// # CFX::CConfigFile::getNumSections()                                  # //
// ####################################################################### //
/** Get number of variables in the config file

    @return The total number of variables
*/
unsigned long CConfigFile::getNumSections(void) const {
  return m_pSections->size();
}

// ####################################################################### //
// # CFX::CConfigFile::getSection()                                      # //
// ####################################################################### //
/** Retrieve a variable from the file

    @param  lIndex  Index of the variable to retrieve
    @return The variable's index
*/
CConfigSection &CConfigFile::getSection(unsigned long nIndex) {
  if(nIndex >= m_pSections->size())
    throw std::out_of_range("The specified index is out of range");

  return m_pSections->operator[](nIndex);
}

// ####################################################################### //
// # CFX::CConfigFile::getSection()                                      # //
// ####################################################################### //
/** Retrieve a variable from the file

    @param  sName  Name of the variable to retrieve
    @return The variable's index
*/
CConfigSection &CConfigFile::getSection(const std::string &sName) {
  SectionVector::iterator SectionEnd = m_pSections->end();
  for(SectionVector::iterator SectionIt = m_pSections->begin();
      SectionIt != SectionEnd;
      SectionIt++) {
    if(SectionIt->getName() == sName)
      return *SectionIt;
  }

  throw std::invalid_argument("A variable with the specified name could not be found");
}

// ####################################################################### //
// # CFX::CConfigFile::addSection()                                      # //
// ####################################################################### //
/** Add a variable to the file

    @param  pSection  Section to add
    @return The variable's index
*/
void CConfigFile::addSection(const CConfigSection &Section) {
  m_pSections->push_back(Section);

  return;
}

// ####################################################################### //
// # CFX::CConfigFile::removeSection()                                   # //
// ####################################################################### //
/** Remove a variable from the file

    @param  lIndex  Index of the variable to remove
*/
void CConfigFile::removeSection(unsigned long lIndex) {
  if(lIndex >= m_pSections->size())
    throw std::out_of_range("The specified index is out of range");

  m_pSections->erase(m_pSections->begin() + lIndex);

  return;
}

// ####################################################################### //
// # CFX::CConfigFile::removeSection()                                   # //
// ####################################################################### //
/** Remove a variable from the file

    @param  sName  Name of the variable to remove
*/
void CConfigFile::removeSection(const std::string &sName) {
  bool bFound = false;

  SectionVector::iterator SectionEnd = m_pSections->end();
  for(SectionVector::iterator SectionIt = m_pSections->begin();
      SectionIt != SectionEnd;
      SectionIt++) {
    if(SectionIt->getName() == sName) {
      m_pSections->erase(SectionIt);
      bFound = true;
    }
  }

  if(!bFound)
    throw std::invalid_argument("A variable with the specified name could not be found");
}


void CConfigFile::ReadConfigInfo( SOFTGPS_config *cfg)
{
  bool corr_flag,source_flag, cntl_flag;

  corr_flag=source_flag=cntl_flag = false;

  int sec_num = getNumSections();
  for( int i=0; i<sec_num; i++)
    {
      // check availablity of different sections
      // if we add additional section in the config.ini, we need change here accordingly
      if( getSection(i).getName() == "SOURCE" )
	source_flag = true;
      else if(getSection(i).getName() == "CORRELATOR")
	corr_flag = true;
      else if(getSection(i).getName() == "CONTROLLER")
	cntl_flag = true;
      else  // for sections we don't care
	{;}
    }
  // next process available sections
  
  if( source_flag )
    {
      bool source_file_flag,sampling_freq_flag,init_phase_flag;
      

      //init all flags to false
      source_file_flag=sampling_freq_flag=init_phase_flag = false;
      
      // first initialize to default value
      cfg->srce_config.source_file = "./data/gpssignal.dat";
      cfg->srce_config.sampling_freq = 5e6;
      cfg->srce_config.init_ph = 0;
      
      int num_var = getSection("SOURCE").getNumVariables();
      
      for( int i=0; i<num_var; i++)
	{
	  if( getSection("SOURCE").getVariable(i).getName() == "SOURCE_FILE" )
	    source_file_flag = true;
	  if( getSection("SOURCE").getVariable(i).getName() == "SAMPLING_FREQ" )
	    sampling_freq_flag = true;
	  if( getSection("SOURCE").getVariable(i).getName() == "INIT_PHASE" )
	    init_phase_flag = true;
	}
      
      if( source_file_flag )
	cfg->srce_config.source_file = getSection("SOURCE").getVariable("SOURCE_FILE").getString();
      else
	cerr<<"No source file setting for SOURCE_FILE, default value used"<< endl;
      
      if( sampling_freq_flag )
	cfg->srce_config.sampling_freq = getSection("SOURCE").getVariable("SAMPLING_FREQ").getDouble();
      else
	cerr<<"No config info for SAMPLING_FREQ, default value "<<cfg->srce_config.sampling_freq<<" used"<< endl;

      if( init_phase_flag )
	cfg->srce_config.init_ph= getSection("SOURCE").getVariable("INIT_PHASE").getLong();
      else
	cerr<<"No config info for INIT_PHASE, default value "<< cfg->srce_config.init_ph<<" used"<< endl;
    } // end of if( source_flag )

  
  if( corr_flag )
    {
      bool prnnum_flag, sampling_freq_flag, carrier_freq_flag;


      prnnum_flag=sampling_freq_flag=carrier_freq_flag = false;

      cfg->corr_config.prnnum = 1;
      cfg->corr_config.sampling_freq = 5e6;
      cfg->corr_config.carrier_freq = 1.25e6;
      
      
      int num_var = getSection("CORRELATOR").getNumVariables();
      

      for( int i=0; i<num_var; i++)
	{
	  if( getSection("CORRELATOR").getVariable(i).getName() == "PRNNUM" )
	    prnnum_flag = true;
	  if( getSection("CORRELATOR").getVariable(i).getName() == "SAMPLING_FREQ" )
	    sampling_freq_flag = true;
	  if( getSection("CORRELATOR").getVariable(i).getName() == "CARRIER_FREQ" )
	    carrier_freq_flag = true;
	}
      
      if( prnnum_flag )
	cfg->corr_config.prnnum = 
	  getSection("CORRELATOR").getVariable("PRNNUM").getLong();
      else
	cerr <<"No config info for PRNNUM, default value " << cfg->corr_config.prnnum << " used " << endl;

      if( sampling_freq_flag )
	cfg->corr_config.sampling_freq = 
	  getSection("CORRELATOR").getVariable("SAMPLING_FREQ").getDouble();
      else
	cerr<<"No config info for SAMPLING_FREQ, default value "<<cfg->corr_config.sampling_freq<<" used"<< endl;


      if( carrier_freq_flag )
	cfg->corr_config.carrier_freq = getSection("CORRELATOR").getVariable("CARRIER_FREQ").getDouble();
      else
	cerr<<"No config info for CARRIER_FREQ, default value "<<cfg->corr_config.carrier_freq<<" used"<< endl;
      
    } // end of if( corr_flag )

  if( cntl_flag )
    {
      bool threshold_flag, prnlist_flag, cafreqlist_flag, cdphase_flag;
      
      threshold_flag = prnlist_flag = cafreqlist_flag= cdphase_flag = false;
      
      cfg->cntl_config.threshold = 500.0;
      cfg->cntl_config.prnlist = "";
      cfg->cntl_config.ca_freqlist="";
      cfg->cntl_config.cd_phaselist = "";

      int num_var = getSection("CONTROLLER").getNumVariables();
      

      for( int i=0; i<num_var; i++)
	{
	  if( getSection("CONTROLLER").getVariable(i).getName() == "THRESHOLD" )
	    threshold_flag = true;
	  if( getSection("CONTROLLER").getVariable(i).getName() == "CHPRN" )
	    prnlist_flag = true;
	  if( getSection("CONTROLLER").getVariable(i).getName() == "CARRIER" )
	    cafreqlist_flag = true;
	  if( getSection("CONTROLLER").getVariable(i).getName() == "CDPHASE")
	    cdphase_flag = true;


	}
      if( threshold_flag )
	cfg->cntl_config.threshold = 
	  getSection("CONTROLLER").getVariable("THRESHOLD").getDouble();
      else
	cerr<<"No config info for THRESHOLD, default value "<<cfg->cntl_config.threshold<<" used"<< endl;

      if( prnlist_flag )
	cfg->cntl_config.prnlist = getSection("CONTROLLER").getVariable("CHPRN").getString();
      else
	cerr<<"No config info for CHPRN, default value used"<< endl;

      if( cafreqlist_flag )
	cfg->cntl_config.ca_freqlist = getSection("CONTROLLER").getVariable("CARRIER").getString();
      else
	cerr<<"No config info for CARRIER, default value used"<< endl;
      
      if( cdphase_flag )
	cfg->cntl_config.cd_phaselist = getSection("CONTROLLER").getVariable("CDPHASE").getString();
      else
	cerr<<"No config info for CDPHASE, default value used"<< endl;
      

    } // end of if(cntl_flag )
  
}

void CConfigFile::PrintoutConfigInfo( const SOFTGPS_config& cfg)
{
  cout << "[SOURCE]"<< endl;

  cout << "SOURCE_FILE=" << cfg.srce_config.source_file << endl;
  cout << "SAMPLING_FREQ="<< cfg.srce_config.sampling_freq << endl;
  cout << "INIT_PHASE=" << cfg.srce_config.init_ph << endl;

  cout << "[CORRELATOR]" <<endl;
  cout << "PRNNUM=" << cfg.corr_config.prnnum<< endl;
  cout << "SAMPLING_FREQ=" << cfg.corr_config.sampling_freq<< endl;
  cout << "CARRIER_FREQ=" << cfg.corr_config.carrier_freq << endl;
  
  cout <<"[CONTROLLER]" <<endl;
  cout <<"THRESHOLD=" << cfg.cntl_config.threshold<<endl;
  cout <<"CHPRN=" << cfg.cntl_config.prnlist<<endl;
  cout <<"CARRIER=" <<cfg.cntl_config.ca_freqlist<<endl;
  cout <<"CDPHASE=" << cfg.cntl_config.cd_phaselist<<endl;

}




      

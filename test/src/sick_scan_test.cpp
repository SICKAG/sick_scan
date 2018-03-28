/*
* Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2018, SICK AG, Waldkirch
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
*     * Neither the name of Osnabr�ck University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
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
*  Last modified: 27th Mar 2018
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "boost/filesystem.hpp"
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <sys/wait.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>


#include "sick_scan/sick_generic_laser.h"

#include <ros/console.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#include "tinystr.h"
#include "tinyxml.h"

#define MAX_NAME_LEN (1024)

// 001.000.000 Initial Version
#define SICK_SCAN_TEST_MAJOR_VER "001"
#define SICK_SCAN_TEST_MINOR_VER "000"  
#define SICK_SCAN_TEST_PATCH_LEVEL "000"

#include <algorithm> // for std::min


class paramEntryAscii
{
public:
	paramEntryAscii(std::string _nameVal, std::string _typeVal, std::string _valueVal)
	{
		nameVal = _nameVal;
		typeVal = _typeVal;
		valueVal = _valueVal;
    setCheckStatus(999,"untested");
	};

  void setPointerToXmlNode(TiXmlElement *paramEntryPtr)
  {
    this->nodePtr = paramEntryPtr;
  }

    TiXmlElement * getPointerToXmlNode(void)
    {
      return( this->nodePtr);
    }
	void setValues(std::string _nameVal, std::string _typeVal, std::string _valueVal)
	{
		nameVal = _nameVal;
		typeVal = _typeVal;
		valueVal = _valueVal;
	};

	std::string getName()
	{
		return(nameVal);
	}

	std::string getType()
	{
		return(typeVal);
	}

	std::string getValue()
	{
		return(valueVal);
	}

  void setCheckStatus(int errCode, std::string errMsg)
  {
    errorCode = errCode;
    errorMsg = errMsg;
  };

    int getErrorCode()
    {
      return(errorCode);
    }

    std::string getErrorMsg()
    {
      return(errorMsg);
    }

private:
	std::string nameVal;
	std::string typeVal;
	std::string valueVal;
  int errorCode;
  std::string errorMsg;
    TiXmlElement *nodePtr;
};





void sudokill(pid_t tokill)
{

  kill(tokill, SIGTERM);
#if 0
  char *killstr;
  int status;
  pid_t pid;

  if (asprintf(&killstr, "%d", tokill) < 0)
    printf("asprintf() failed");

  pid = fork();
  switch (pid)
  {
    case -1:
      printf( "fork() failed");
    case 0:
      execlp("sudo", "sudo", "kill", killstr, (char *) NULL);
      printf( "execlp() failed");
    default:
      while (wait(&status) != pid);
  }

  free(killstr);
#endif
  sleep(5);
}

std::vector<paramEntryAscii> getParamList(TiXmlNode *paramList)
{
	std::vector<paramEntryAscii> tmpList;


	TiXmlElement *paramEntry = (TiXmlElement *)paramList->FirstChild("param");
	while (paramEntry)
	{
		std::string nameVal = "";
		std::string typeVal = "";
		std::string valueVal = "";

		for (TiXmlAttribute* node = paramEntry->FirstAttribute(); ; node = node->Next())
		{
			const char *tag = node->Name();
			const char *val = node->Value();

			if (strcmp(tag, "name") == 0)
			{
				nameVal = val;
			}
			if (strcmp(tag, "type") == 0)
			{
				typeVal = val;
			}
			if (strcmp(tag, "value") == 0)
			{
				valueVal = val;
			}
			if (node == paramEntry->LastAttribute())
			{

				break;
			}
		}

		paramEntryAscii tmpEntry(nameVal, typeVal, valueVal);
    tmpEntry.setPointerToXmlNode(paramEntry);
		tmpList.push_back(tmpEntry);
		paramEntry = (TiXmlElement *)paramEntry->NextSibling();
	}

	return(tmpList);
}



void replaceParamList(TiXmlNode * node, std::vector<paramEntryAscii> paramOrgList)
{
	bool fndParam = false;
	do
	{
		fndParam = false;
		TiXmlElement *paramEntry = (TiXmlElement *)node->FirstChild("param");
		if (paramEntry != NULL)
		{
			fndParam = true;
			node->RemoveChild(paramEntry);
		}
	} while (fndParam == true);

	for (int i = 0; i < paramOrgList.size(); i++)
	{
		paramEntryAscii tmp = paramOrgList[i];
		TiXmlElement *paramTmp = new TiXmlElement("param");
		paramTmp->SetAttribute("name", tmp.getName().c_str());
		paramTmp->SetAttribute("type", tmp.getType().c_str());
		paramTmp->SetAttribute("value", tmp.getValue().c_str());
		node->LinkEndChild(paramTmp);
	}
}

pid_t pid;

pid_t launchRosFile(std::string cmdLine)
{
  std::string cmd = cmdLine;
  typedef std::vector<std::string > split_vector_type;

  split_vector_type splitVec; // #2: Search for tokens
  boost::split(splitVec, cmdLine, boost::is_any_of(" "), boost::token_compress_on); // SplitVec == { "hello abc","ABC","aBc goodbye" }



  pid_t pid = fork(); // create child process
  int status;

  switch (pid)
  {
    case -1: // error
      perror("fork");
      exit(1);

    case 0: // child process
      execl("/opt/ros/kinetic/bin/roslaunch", splitVec[0].c_str(), splitVec[1].c_str(),  splitVec[2].c_str(), NULL); // run the command
      perror("execl"); // execl doesn't return unless there is a problem
      exit(1);

    default: // parent process, pid now contains the child pid
  /*
      while (-1 == waitpid(pid, &status, 0)); // wait for child to complete
      if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
      {
        // handle error
        std::cerr << "process " << cmd << " (pid=" << pid << ") failed" << std::endl;
      }
      */
      break;

  }

  return(pid);
}

int createTestLaunchFile(std::string launchFileFullName, std::vector<paramEntryAscii> entryList, std::string& testLaunchFile)
{
	TiXmlDocument doc;
	doc.LoadFile(launchFileFullName.c_str());

	if (doc.Error() == true)
	{
		ROS_WARN("ERROR parsing launch file %s\nRow: %d\nCol: %d", doc.ErrorDesc(), doc.ErrorRow(), doc.ErrorCol());
	}
	TiXmlNode *node = doc.FirstChild("launch");
	if (node != NULL)
	{
		node = node->FirstChild("node");
		std::vector<paramEntryAscii> paramOrgList = getParamList(node);
		for (int i = 0; i < entryList.size(); i++)
		{
			bool replaceEntry = false;
			for (int j = 0; j < paramOrgList.size(); j++)
			{
				if (entryList[i].getName().compare(paramOrgList[j].getName()) == 0)
				{
					paramOrgList[j].setValues(entryList[i].getName(), entryList[i].getType(), entryList[i].getValue());
					replaceEntry = true;
				}
			}
			if (replaceEntry == false)  // add new one to list
			{
				paramEntryAscii tmp(entryList[i].getName(), entryList[i].getType(), entryList[i].getValue());
				paramOrgList.push_back(tmp);
			}
		}
		// delete all parameters and replace with new one
		replaceParamList(node, paramOrgList);
	}

	// append source extension if any 
	size_t pos = launchFileFullName.rfind('.');
	std::string resultFileName = "";
	testLaunchFile = "";
	if (pos != std::string::npos)
	{
		resultFileName = launchFileFullName.substr(0, pos);
		resultFileName += "_test.launch";
		doc.SaveFile(resultFileName.c_str());
		testLaunchFile = resultFileName;
	}
	return(0);
}

int cloud_width = 0;  // hacky 
int cloud_height = 0;
int callbackCnt = 0;
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	static int cnt = 0;
  callbackCnt++;
	cloud_width = cloud_msg->width;
	cloud_height = cloud_msg->height;
	ROS_INFO("inside callback");

}

int startCallbackTest(int argc, char** argv)
{
	ros::init(argc, argv, "cloudtester");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Subscriber sub;
  ROS_INFO("Cloudtester started."
                  );
	sub = nh.subscribe("cloud", 1, cloud_callback);
  while (callbackCnt < 10)
  {
  ros::spinOnce();
  }
  sub.shutdown();
  ros::shutdown();

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	return(0);

}


void createPrefixFromExeName(std::string exeName, std::string& prefix)
{
#ifdef _MSC_VER
#else
#endif
	std::string searchPattern = "(.*)cmake\\-build\\-debug.*";
	boost::regex expr(searchPattern);
	boost::smatch what;

	if (boost::regex_search(exeName, what, expr))
	{
		std::string prefixPath = what[1];  // started from CLION IDE - build complete path
		prefix = prefixPath;
		prefix += "test";
	}
	else
	{
		prefix = ".";
	}

}


void extractPackagePath(std::string pathList, std::string& packagePath)
{
	typedef std::vector<std::string > split_vector_type;
	packagePath = "???";
	split_vector_type splitVec; // #2: Search for tokens
	boost::split(splitVec, pathList, boost::is_any_of(":"), boost::token_compress_on); // SplitVec == { "hello abc","ABC","aBc goodbye" }
	if (splitVec.size() > 0)
	{
		packagePath = splitVec[0];
	}
}

/*!
\brief Startup routine
\param argc: Number of Arguments
\param argv: Argument variable
\return exit-code
*/
int main(int argc, char **argv)
{
	int result = 0;
	char sep = boost::filesystem::path::preferred_separator;
	std::string prefix = ".";

	std::string exeName = argv[0];

  if (argc == 1)
  {
    printf("Usage: %s <XML-Test-Ctrl-File>\n", exeName.c_str());
    printf("\n");
    printf("The XML-Test-Ctrl-File controls the parameter checking of various scanner types.");
    printf("The XML-File sick_scan_test.xml in the test-Directory shows an example of this file type.\n");
    exit(-1);
  }
#if 0
	createPrefixFromExeName(exeName, prefix);
#ifdef _MSC_VER
	prefix = std::string("..") + sep + std::string("..") + sep + "test";
#endif
	ROS_INFO("sick_scan_test V. %s.%s.%s", SICK_SCAN_TEST_MAJOR_VER, SICK_SCAN_TEST_MINOR_VER, SICK_SCAN_TEST_PATCH_LEVEL);
#endif


	char* pPath;
	pPath = getenv("ROS_PACKAGE_PATH");
#ifdef _MSC_VER
	pPath = "..\\..:\tmp";
#endif
	if (pPath != NULL)
	{
		ROS_INFO("Current root path entries for launch file search: %s\n", pPath);
	}
	else
	{
		ROS_FATAL("Cannot find ROS_PACKAGE_PATH - please run <Workspace>/devel/setup.bash");
	}

	std::string packagePath;
	extractPackagePath(pPath, packagePath);

//	std::string testCtrlXmlFileName = packagePath + std::string(1, sep) + "sick_scan" + std::string(1, sep) + "test" + std::string(1, sep) + "sick_scan_test.xml";
//  $ROS_PACKAGE_PATH/sick_scan/test/sick_scan_test.xml
  std::string testCtrlXmlFileName = argv[1];
  boost::replace_all(testCtrlXmlFileName, "$ROS_PACKAGE_PATH", packagePath);

  TiXmlDocument doc;
	doc.LoadFile(testCtrlXmlFileName.c_str());
	if (doc.Error())
	{
		ROS_ERROR("Cannot load/parse %s", testCtrlXmlFileName.c_str());
		ROS_ERROR("Details: %s\n", doc.ErrorDesc());
		exit(-1);
	}

	boost::filesystem::path p(testCtrlXmlFileName);
	boost::filesystem::path parentPath = p.parent_path();
	std::string pathName = parentPath.string();
	std::string launchFilePrefix = parentPath.parent_path().string() + std::string(1, sep) + "launch" + std::string(1, sep);
	TiXmlNode *node = doc.FirstChild("launch");
	if (node == NULL)
	{
		ROS_ERROR("Cannot find tag <launch>\n");
		exit(-1);
	}

	if (node)
	{
		TiXmlElement *fileNameEntry;
		fileNameEntry = (TiXmlElement *)node->FirstChild("filename");
		if (fileNameEntry == NULL)
		{
			ROS_ERROR("Cannot find tag <filename>\n");
			exit(-1);
		}
		TiXmlText *fileNameVal = (TiXmlText *)fileNameEntry->FirstChild();
		if (fileNameVal != NULL)
		{
			ROS_INFO("Processing launch file %s\n", fileNameVal->Value());
			std::string launchFileFullName = launchFilePrefix + fileNameVal->Value();
			ROS_INFO("Processing %s\n", launchFileFullName.c_str());
			TiXmlNode *paramListNode = (TiXmlNode *)node->FirstChild("paramList");
			if (paramListNode != NULL)
			{
				std::vector<paramEntryAscii> paramList = getParamList(paramListNode);
				std::string testLaunchFile;
				createTestLaunchFile(launchFileFullName, paramList, testLaunchFile);
				std::string commandLine;
				boost::filesystem::path p(testLaunchFile);
				std::string testOnlyLaunchFileName = p.filename().string();
				commandLine = "roslaunch sick_scan " + testOnlyLaunchFileName;
				ROS_INFO("launch ROS test ... [%s]", commandLine.c_str());

        // int result = std::system(commandLine.c_str());

        pid_t pidId = launchRosFile(commandLine.c_str());
				startCallbackTest(argc, argv); // get 10 pointcloud messages 

        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
				ROS_INFO("If you receive an error message like \"... is neither a launch file ... \""
					"please source ROS env. via source ./devel/setup.bash");


				TiXmlNode *resultListNode = (TiXmlNode *)node->FirstChild("resultList");
				if (resultListNode != NULL)
				{
					std::vector<paramEntryAscii> resultList = getParamList(resultListNode);
					for (int i = 0; i < resultList.size(); i++)
					{
						if (resultList[i].getName().compare("shotsPerLayer") == 0)
						{
							int expectedWidth = -1;
							std::string valString = resultList[i].getValue();
							const char *valPtr = valString.c_str();
							if (valPtr != NULL)
							{
								sscanf(valPtr, "%d", &expectedWidth);
							}

              ROS_INFO("Test\n");
              int errorCode = 0;
							if (expectedWidth == cloud_width)
							{
								printf("CHECK PASSED! WIDTH %d == %d\n", expectedWidth, cloud_width);
                errorCode = 0;
							}
							else
							{
                printf("CHECK FAILED! WIDTH %d <> %d\n", expectedWidth, cloud_width);
                errorCode = 1;

							}
              resultList[i].setCheckStatus(errorCode, (errorCode == 0) ? "OK" : "Unexpected number of shots");
              cloud_width = 0;
						}

            TiXmlElement *paramSet = resultList[i].getPointerToXmlNode();
            paramSet->SetAttribute("errorCode", resultList[i].getErrorCode());
            paramSet->SetAttribute("errorMsg", resultList[i].getErrorMsg().c_str());
					}

				}

        printf("Killing process %d\n", pidId);
        sudokill(pidId);

        std::string testCtrlResultXmlFileName = "";
        size_t pos = testCtrlXmlFileName.rfind('.');
        if (pos != std::string::npos)
        {
          testCtrlResultXmlFileName = testCtrlXmlFileName.substr(0,pos) + "_res.xml";
          doc.SaveFile(testCtrlResultXmlFileName.c_str());
        }

			}
		}
		else
		{
			ROS_WARN("Cannot find filename entry");
		}


	}
	return result;

}


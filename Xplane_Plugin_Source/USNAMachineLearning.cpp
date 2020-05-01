/** 
   The USNA XPlane Machine Learning Plugin
   writen by Midshipmen 1/C Yates
   project advisor Dr. Gavin Taylor
   
   	
 */

#include "XPLMPlugin.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"
#include "XPLMScenery.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "tinyxml.h"
#include <vector>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
using namespace std;

// needed constants
const float Pi = 3.14159265358979;
const float RadToDeg = 180.0 / Pi;
const float DegToRad = 1.0 / RadToDeg;



float stepsum = 0;
int xplround =0;
double timeFactor =0;
double runwayElevation =0;
bool isFirst = true;

/// trim from start
string &ltrim(string &s) {
        s.erase(s.begin(), find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(isspace))));
        return s;
}

/// trim from end
string &rtrim(string &s) {
        s.erase(find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(isspace))).base(), s.end());
        return s;
}

/// trim from both ends
string &trim(string &s) {
        return ltrim(rtrim(s));
}


/// turns a number into a string
string convertInt(int number)
{
   stringstream ss;
   ss << number;
   return ss.str();
}

/// makes a float a string
string convertFloat(float number)
{
   stringstream ss;
   ss << number;
   return ss.str();
}

/// puts a sting in a file
void  stringToFile(string fileName, string txtToWrite) 
{
	std::ofstream outfile;
	outfile.open(fileName.c_str(), std::ios_base::app);
	outfile << txtToWrite; 
	outfile.close();
}

/// takes a file name and converts it into a vector of strings one for each line
vector<string> fileToVectorOfStrings(string file_name)
{
    ifstream infile(file_name.c_str());
    string line;
    vector<string> out;
    while (std::getline(infile, line))
    {
        out.push_back(trim(line));
    }
    return out;
}

/// data types for position that takes 4 velocities
typedef struct _QUATERNION
{
	float w;
	float x;
	float y;
	float z;
} QUATERNION;

/// used to hold the three degrees
typedef struct _HPR
{
	float Heading;
	float Pitch;
	float Roll;
} HPR;




/// converts the 4 element quaternion into roll pitch and yaw
void QuaternionToHPR(QUATERNION quaternion, HPR *phpr)
{
	double local_w = quaternion.w;
	double local_x = quaternion.x;
	double local_y = quaternion.y;
	double local_z = quaternion.z;

	double sq_w = local_w * local_w;
	double sq_x = local_x * local_x;
	double sq_y = local_y * local_y;
	double sq_z = local_z * local_z;

	phpr->Heading = atan2(2.0 * (local_x * local_y + local_z * local_w),
			(sq_x - sq_y - sq_z + sq_w)) * RadToDeg;
	phpr->Pitch = asin(-2.0 * (local_x * local_z - local_y * local_w)) 
		* RadToDeg;
	phpr->Roll = atan2(2.0 * (local_y * local_z + local_x * local_w),
			(-sq_x - sq_y + sq_z + sq_w)) * RadToDeg;
}

/// converts roll pitch and yaw back into a quaternion
void HPRToQuaternion(HPR hpr, QUATERNION *pquaternion)
{
	double local_Heading = hpr.Heading * DegToRad;
	double local_Pitch = hpr.Pitch * DegToRad;
	double local_Roll = hpr.Roll * DegToRad;

	double Cosine1 = cos(local_Roll / 2);
	double Cosine2 = cos(local_Pitch / 2);
	double Cosine3 = cos(local_Heading / 2); 
	double Sine1 = sin(local_Roll / 2);
	double Sine2 = sin(local_Pitch / 2);
	double Sine3 = sin(local_Heading / 2);

	pquaternion->w = Cosine1 * Cosine2 * Cosine3 + Sine1 * Sine2 * Sine3;
	pquaternion->x = Sine1 * Cosine2 * Cosine3 - Cosine1 * Sine2 * Sine3;
	pquaternion->y = Cosine1 * Sine2 * Cosine3 + Sine1 * Cosine2 * Sine3;
	pquaternion->z = Cosine1 * Cosine2 * Sine3 - Sine1 * Sine2 * Cosine3;
}


/// the datapoint class is the wrapper for datarefs and for 
/// custom values that are a little confusing to get out of Xplane
class DataPoint {
	public:
	string name;
	DataPoint (string ref)
	{
		name = ref;
	}
	void set(float in)
	{
		if(name.compare("speed")==0 || name.compare("time")==0)
		{}
		else if(name.compare("elevation")==0)
		{
			XPLMSetDataf(XPLMFindDataRef("sim/flightmodel/position/elevation"),in);
		}
		else if(name.compare("pitch")==0)
		{
			QUATERNION q1;
			HPR hpr;
			float FloatVals[4];
			int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
			q1.w = FloatVals[0];
			q1.x = FloatVals[1];
			q1.y = FloatVals[2];
			q1.z = FloatVals[3];
			QuaternionToHPR(q1, &hpr);
			hpr.Pitch = in;
			HPRToQuaternion(hpr, &q1);
			FloatVals[0] = q1.w;
			FloatVals[1] = q1.x;
			FloatVals[2] = q1.y;
			FloatVals[3] = q1.z;
			XPLMSetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals,0,4);
		}
		else if(name.compare("yaw")==0)
		{
			QUATERNION q1;
			HPR hpr;
			float FloatVals[4];
			int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
			q1.w = FloatVals[0];
			q1.x = FloatVals[1];
			q1.y = FloatVals[2];
			q1.z = FloatVals[3];
			QuaternionToHPR(q1, &hpr);
			hpr.Heading = in;
			HPRToQuaternion(hpr, &q1);
			FloatVals[0] = q1.w;
			FloatVals[1] = q1.x;
			FloatVals[2] = q1.y;
			FloatVals[3] = q1.z;
			XPLMSetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals,0,4);
		}
		else if(name.compare("roll")==0)
		{
			QUATERNION q1;
			HPR hpr;
			float FloatVals[4];
			int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
			q1.w = FloatVals[0];
			q1.x = FloatVals[1];
			q1.y = FloatVals[2];
			q1.z = FloatVals[3];
			QuaternionToHPR(q1, &hpr);
			hpr.Roll = in;
			HPRToQuaternion(hpr, &q1);
			FloatVals[0] = q1.w;
			FloatVals[1] = q1.x;
			FloatVals[2] = q1.y;
			FloatVals[3] = q1.z;
			XPLMSetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals,0,4);
		}
		else
		{
			XPLMSetDataf(XPLMFindDataRef(name.c_str()),in);

		}
		XPLMFindDataRef("sim/flightmodel/position/q");
		QUATERNION q1;
		HPR hpr;
		float FloatVals[4];

		int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
		q1.w = FloatVals[0];
		q1.x = FloatVals[1];
		q1.y = FloatVals[2];
		q1.z = FloatVals[3];
		QuaternionToHPR(q1, &hpr);
	}

	double get() 
	{
		if(name.compare("time")==0)
		{
			return XPLMGetElapsedTime()- timeFactor;
		}
		else if(name.compare("speed")==0)
		{
			double vx = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/local_vx"));
			double vy = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/local_vy"));
			double vz = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/local_vz"));
			return sqrt((vx * vx) + (vy * vy) + (vz * vz));	
		}
		else if(name.compare("elevation")==0)
		{
			return XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/elevation"))- runwayElevation;
		}
		else if(name.compare("pitch")==0)
		{
			QUATERNION q1;
			HPR hpr;
			float FloatVals[4];

			int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
			q1.w = FloatVals[0];
			q1.x = FloatVals[1];
			q1.y = FloatVals[2];
			q1.z = FloatVals[3];
			QuaternionToHPR(q1, &hpr);
			return hpr.Pitch;
		}
		else if(name.compare("yaw")==0)
		{
			QUATERNION q1;
			HPR hpr;
			float FloatVals[4];

			int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
			q1.w = FloatVals[0];
			q1.x = FloatVals[1];
			q1.y = FloatVals[2];
			q1.z = FloatVals[3];
			QuaternionToHPR(q1, &hpr);
			return hpr.Heading;
		}
		else if(name.compare("roll")==0)
		{
			QUATERNION q1;
			HPR hpr;
			float FloatVals[4];

			int count = XPLMGetDatavf(XPLMFindDataRef("sim/flightmodel/position/q"), FloatVals, 0, 4);
			q1.w = FloatVals[0];
			q1.x = FloatVals[1];
			q1.y = FloatVals[2];
			q1.z = FloatVals[3];
			QuaternionToHPR(q1, &hpr);
			return hpr.Roll;
		}
		else
		{
			return XPLMGetDataf(XPLMFindDataRef(name.c_str()));
		}
	}
};
vector<string> cmdKeyArray;
/// builds a number for what key name goes with what number
void makecmdKeyArray() 
{  	// 95 elements
	cmdKeyArray = fileToVectorOfStrings("XPLMCommandKeyID.lst");
	return;
}
vector<string> cmdButtonArray;
// builds a vector for looking up what number goes with what
// button
void makecmdButtonArray()
{	
cmdButtonArray = fileToVectorOfStrings("XPLMCommandButtonID.lst");
	return;
}
// converts human readable strings to ints that Xplane can use
int findButtonInt(string buttonToBePressed)
{
	for(int ii=0; ii<buttonToBePressed.size(); ii++)
	{
		if(buttonToBePressed.compare(cmdButtonArray[ii])==0)
			return ii;
	}
	return -1;
}
// the XMl will read in the name of the keys
// but Xplane needs enums for to interact with
int findKeyInt(string keyToBePressed)
{
	for(int ii=0; ii<keyToBePressed.size(); ii++)
	{
		if(keyToBePressed.compare(cmdKeyArray[ii])==0)
			return ii;
	}
	return -1;
}

//file to be read 
string xmlFileName = "config.xml"; 
string pluginName = "Some random plugin name that was not set by the XML";
double recordRate = 1;
double stepRate = 1;
bool reloadEveryRun = true;
bool reloadOnChange = true;
bool debug = false;
bool isStart = true;
string outFile = "notARealFile.txt";
bool writeout = false;
// Global Varibles needed that hold the lists of datarefs
// everytime the plane resets on the runway these lists apply these value
vector<string> datarefsThatGetResetPerRun;
vector<DataPoint> dpPerRun;
vector<double> datarefsThatGetResetPerRunValues;
// values that every record step are reset
vector<string> datarefsThatGetResetPerRecord;
vector<DataPoint> dpPerRecord;
vector<double> datarefsThatGetResetPerRecordValues;
// vales that are reset every step
vector<string> datarefsThatGetResetPerStep;
vector<DataPoint> dpPerStep;
vector<double> datarefsThatGetResetPerStepValues;
// these are writen to standout every record
vector<string> datarefsThatGetMeasured;
vector<DataPoint> dpMeasured;
// these are the values that need to be changed every step 
vector<string> datarefsThatGetChanged;
vector<DataPoint> dpChanged;
vector<vector<double> > valuesThatCanBeChosen;
// these are the learned datarefs every record
vector<string> datarefsThatHaveBeenLearned;
vector<DataPoint> dpLearned;
vector<vector<string> > datarefsReadForTheDecisions; 
vector<vector<DataPoint> > dpRead;
vector<vector<vector<double> > > weightsForLearnedDatarefs;
vector<vector<double> > valuesThatCanBeChosenForLearnedDatarefs;
// buttons that need to be pressed
vector<int> buttonsPressedPerRun;
vector<int> buttonsPressedPerRecord;
vector<int> buttonsPressedPerStep;
// keys that need to be pressed
vector<int> keysPressedPerRun;
vector<int> keysPressedPerRecord;
vector<int> keysPressedPerStep;

/// prints everything being measured on one line 
void makeMeasurements()
{
	for (int ii =0; ii< dpMeasured.size(); ii++)
		cout <<dpMeasured[ii].get() << " ";
}

/// randomly makes a choice from the allowed choices for each and every sample positions
void makeGuesses()
{
	makeMeasurements();
	for (int ii =0; ii< dpChanged.size(); ii++)
	{
		int temp = rand() % valuesThatCanBeChosen[ii].size();
		dpChanged[ii].set(valuesThatCanBeChosen[ii][temp]);
		cout << temp << " ";
	}
	cout << "\n";

}

/// prints measurement by measurement
void makeMeasurements(string outf)
{
    for (int ii =0; ii< dpMeasured.size(); ii++)
        stringToFile( outf.c_str(),convertFloat(dpMeasured[ii].get()) + " ");
}

/// used to pick the random action
void makeGuesses(string outf)
{
	makeMeasurements(outf);
	for (int ii =0; ii< dpChanged.size(); ii++)
	{
		int temp = rand() % valuesThatCanBeChosen[ii].size();
		dpChanged[ii].set(valuesThatCanBeChosen[ii][temp]);
		stringToFile( outf.c_str(),convertInt(temp) + " ");
	}
	stringToFile( outf.c_str(),"\n");

}

// calls the button press API
void pressButtons(vector<int> buttons)
{
	for(int ii=0; ii<buttons.size(); ii++)
	{
		XPLMCommandButtonPress(buttons[ii]);
	}
	return;
}

// calles the key press API
void pressKeys(vector<int> keys)
{
	for(int ii=0; ii<keys.size(); ii++)
	{
		XPLMCommandKeyStroke(keys[ii]);
	}
	return;
}


// go through the datapoints that are read for every learned component
// and takes the measued value and multiplies it by the measured weight
// then it takes the sum of these products and if that sum is higher than
// the sum for any of the other positions it takes that choice
void makeChoices()
{
	double bestResult, result;
	int choice;
	for(int ii =0; ii< dpLearned.size(); ii++)
	{
		bestResult =0;
		for(int kk =0; kk< valuesThatCanBeChosenForLearnedDatarefs[ii].size(); kk++ )
		{
			result = 0;
			for(int jj =0; jj< dpRead[ii].size(); jj++ )
			{
				result += dpRead[ii][jj].get()*weightsForLearnedDatarefs[ii][kk][jj];
			}	
			if(kk==0 || result>bestResult)
			{
				choice =kk;
				bestResult = result;
			}
		}

		dpLearned[ii].set(valuesThatCanBeChosenForLearnedDatarefs[ii][choice]);
	}	
}

void resetDPs(vector<DataPoint> dps,  vector<double> vals)
{
	for(int ii =0; ii< dps.size(); ii++)
		dps[ii].set(vals[ii]);
}



// ----------------------------------------------------------------------
// The section below was copied and adopted from http://www.grinninglizard.com/tinyxmldocs/index.html
// It is used for seeing the stated of the XML document that is read in
// ----------------------------------------------------------------------
const unsigned int NUM_INDENTS_PER_SPACE=2;

const char * getIndent( unsigned int numIndents )
{
	static const char * pINDENT="                                      + ";
	static const unsigned int LENGTH=strlen( pINDENT );
	unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
	if ( n > LENGTH ) n = LENGTH;

	return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
	static const char * pINDENT="                                        ";
	static const unsigned int LENGTH=strlen( pINDENT );
	unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
	if ( n > LENGTH ) n = LENGTH;

	return &pINDENT[ LENGTH-n ];
}

int dump_attribs_to_stdout(TiXmlElement* pElement, unsigned int indent)
{
	if ( !pElement ) return 0;

	TiXmlAttribute* pAttrib=pElement->FirstAttribute();
	int i=0;
	int ival;
	double dval;
	const char* pIndent=getIndent(indent);
	printf("\n");
	while (pAttrib)
	{
		printf( "%s%s: value=[%s]", pIndent, pAttrib->Name(), pAttrib->Value());

		if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
		if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
		printf( "\n" );
		i++;
		pAttrib=pAttrib->Next();
	}
	return i;	
}

void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0 )
{
	if ( !pParent ) return;

	TiXmlNode* pChild;
	TiXmlText* pText;
	int t = pParent->Type();
	printf( "%s", getIndent(indent));
	int num;

	switch ( t )
	{
		case /*TiXmlNode::DOCUMENT*/ 0:
			printf( "Document" );
			break;

		case /*TiXmlNode::ELEMENT*/ 1:
			printf( "Element [%s]", pParent->Value() );
			num=dump_attribs_to_stdout(pParent->ToElement(), indent+1);
			switch(num)
			{
				case 0:  printf( " (No attributes)"); break;
				case 1:  printf( "%s1 attribute", getIndentAlt(indent)); break;
				default: printf( "%s%d attributes", getIndentAlt(indent), num); break;
			}
			break;

		case /*TiXmlNode::COMMENT*/ 2:
			printf( "Comment: [%s]", pParent->Value());
			break;

		case /*TiXmlNode::UNKNOWN*/ 3:
			printf( "Unknown" );
			break;

		case /*TiXmlNode::TEXT*/ 4:
			pText = pParent->ToText();
			printf( "Text: [%s]", pText->Value() );
			break;

		case /*TiXmlNode::DECLARATION*/ 5:
			printf( "Declaration" );
			break;
		default:
			break;
	}
	printf( "\n" );
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		dump_to_stdout( pChild, indent+1 );
	}
}

/// load the named file and dump its structure to STDOUT
void dump_to_stdout(const char* pFilename)
{
	TiXmlDocument doc(pFilename);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		printf("\n%s:\n", pFilename);
		dump_to_stdout( &doc ); // defined later in the tutorial
	}
	else
	{
		printf("Failed to load file \"%s\"\n", pFilename);
	}
}


// utility to make strings lower case for
// case insensitve comparison
string toLower(string str)
{
	int differ = 'A'-'a';
	for(std::string::size_type i = 0; i < str.size(); ++i) 
	{
		str[i] -= differ;
	}	
	return str;
}

/// converts strings to datapoints
void covertToData()
{
	for(int ii =0; ii< datarefsThatGetResetPerRun.size(); ii++)
	{
		dpPerRun.push_back(DataPoint( datarefsThatGetResetPerRun[ii]));
	}

	for(int ii =0; ii< datarefsThatGetResetPerRecord.size(); ii++)
	{
		dpPerRecord.push_back(DataPoint( datarefsThatGetResetPerRecord[ii]));
	}
	for(int ii =0; ii< datarefsThatGetResetPerRecord.size(); ii++)
	{
		dpPerRecord.push_back(DataPoint( datarefsThatGetResetPerRecord[ii]));
	}
	for(int ii =0; ii< datarefsThatGetResetPerStep.size(); ii++)
	{
		dpPerStep.push_back(DataPoint( datarefsThatGetResetPerStep[ii]));
	}
	
      for(int ii =0; ii< datarefsThatGetMeasured.size(); ii++)
        {
                dpMeasured.push_back(DataPoint( datarefsThatGetMeasured[ii]));
        }

        for(int ii =0; ii< datarefsThatGetChanged.size(); ii++)
        {
                dpChanged.push_back(DataPoint( datarefsThatGetChanged[ii]));
        }

        for(int ii =0; ii< datarefsThatHaveBeenLearned.size(); ii++)
        {
                dpLearned.push_back(DataPoint( datarefsThatHaveBeenLearned[ii]));
                dpRead.push_back(vector<DataPoint>());
                for(int jj =0; jj< datarefsReadForTheDecisions[ii].size(); jj++)
                {
                        dpRead[ii].push_back(datarefsReadForTheDecisions[ii][jj]);
                }
        }
}

// case XML is reloaded would be be bad to fill vetors with more and more stuff
void clearAllGlobalVectors()
{
	datarefsThatGetResetPerRun.clear();
	dpPerRun.clear();
	datarefsThatGetResetPerRunValues.clear();
	datarefsThatGetResetPerRecord.clear();
	dpPerRecord.clear();
	datarefsThatGetResetPerRecordValues.clear();
	datarefsThatGetResetPerStep.clear();
	dpPerStep.clear();
	datarefsThatGetResetPerStepValues.clear();
	datarefsThatGetMeasured.clear();
	dpMeasured.clear();
	datarefsThatGetChanged.clear();
	dpChanged.clear();
	valuesThatCanBeChosen.clear();
	datarefsThatHaveBeenLearned.clear();
	dpLearned.clear();
	datarefsReadForTheDecisions.clear(); 
	dpRead.clear();
	weightsForLearnedDatarefs.clear();
	valuesThatCanBeChosenForLearnedDatarefs.clear();
	buttonsPressedPerRun.clear();
	buttonsPressedPerRecord.clear();
	buttonsPressedPerStep.clear();
	keysPressedPerRun.clear();
	keysPressedPerRecord.clear();
	keysPressedPerStep.clear();
}
// reads in the XML from the passed in filename
void loadXML(const char* pFilename)
{
	clearAllGlobalVectors();
	TiXmlDocument doc(pFilename);
	if (!doc.LoadFile()) return; // if there is no file it breaks
	TiXmlHandle hDoc(&doc); 
	TiXmlHandle hRoot(0);
	TiXmlElement* pElem=hDoc.FirstChildElement().Element();
	// should always have a valid root but handle gracefully if it does
	if (!pElem) return; 
	hRoot=TiXmlHandle(pElem);
	//TiXmlElement* aElement = hRoot.FirstChildElement("learned").ToElement();
	TiXmlElement* aElement =  hRoot.FirstChild("learned").FirstChild().Element();
	TiXmlElement* dfe;
	for(;aElement; aElement=aElement->NextSiblingElement())
	{
		datarefsThatHaveBeenLearned.push_back(aElement->Attribute("dataref"));
		dfe = TiXmlHandle(aElement).FirstChildElement("listref").FirstChildElement().ToElement();
		datarefsReadForTheDecisions.push_back(vector<string>());		
		for(; dfe; dfe=dfe->NextSiblingElement())
		{
			datarefsReadForTheDecisions.back().push_back(dfe->GetText());
		}
		dfe =  TiXmlHandle(aElement).FirstChild("listoptions").FirstChild().Element();
		valuesThatCanBeChosenForLearnedDatarefs.push_back(vector<double>());
		for( ; dfe; dfe=dfe->NextSiblingElement())
		{
			valuesThatCanBeChosenForLearnedDatarefs.back().push_back(atof(dfe->GetText()));
		}  
		dfe =  TiXmlHandle(aElement).FirstChild("listsweights").FirstChild().Element();

		weightsForLearnedDatarefs.push_back(vector< vector<double> >());

		for( ; dfe; dfe=dfe->NextSiblingElement())   
		{  
			TiXmlElement* tfe = TiXmlHandle(dfe).FirstChild().Element();
			weightsForLearnedDatarefs.back().push_back(vector<double>());

			for( ; tfe; tfe=tfe->NextSiblingElement())   
			{
				weightsForLearnedDatarefs.back().back().push_back(atof(tfe->GetText()));
			}
		} 
	}
	
	aElement = hRoot.FirstChild("samples").FirstChild().Element();
	for( ; aElement; aElement=aElement->NextSiblingElement())
	{
		datarefsThatGetChanged.push_back(aElement->Attribute("dataref"));
		dfe =  TiXmlHandle(aElement).FirstChild("listoptions").FirstChild().Element();
		valuesThatCanBeChosen.push_back(vector<double>());
		for(; dfe; dfe=dfe->NextSiblingElement())   
		{  
			valuesThatCanBeChosen.back().push_back(atof(dfe->GetText()));
		}
	}
	aElement = hRoot.FirstChild("stepresets").FirstChild().Element();
	for( ; aElement; aElement=aElement->NextSiblingElement())
	{
		datarefsThatGetResetPerStep.push_back(aElement->Attribute("dataref"));
		datarefsThatGetResetPerStepValues.push_back(atof(aElement->Attribute("value")));
	}
	aElement = hRoot.FirstChild("recordresets").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{
		datarefsThatGetResetPerRecord.push_back(aElement->Attribute("dataref"));
		datarefsThatGetResetPerRecordValues.push_back(atof(aElement->Attribute("value")));
	}
	aElement = hRoot.FirstChild("runresets").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{
		datarefsThatGetResetPerRun.push_back(aElement->Attribute("dataref"));
		datarefsThatGetResetPerRunValues.push_back(atof(aElement->Attribute("value")));
	}
	aElement = hRoot.FirstChild("measures").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
		datarefsThatGetMeasured.push_back(aElement->GetText());

	aElement = hRoot.FirstChild("runbuttons").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{  
		buttonsPressedPerRun.push_back(findButtonInt(aElement->GetText()));
	}

	aElement = hRoot.FirstChild("recordbuttons").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{  
		buttonsPressedPerRecord.push_back(findButtonInt(aElement->GetText()));
	}

	aElement = hRoot.FirstChild("stepbuttons").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{  
		buttonsPressedPerStep.push_back(findButtonInt(aElement->GetText()));
	}

	aElement = hRoot.FirstChild("runkeys").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{  
		keysPressedPerRun.push_back(findKeyInt(aElement->GetText()));
	}

	aElement = hRoot.FirstChild("recordkeys").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{  
		keysPressedPerRecord.push_back(findKeyInt(aElement->GetText()));
	}

	aElement = hRoot.FirstChild("stepkeys").FirstChild().Element();
	for(; aElement; aElement=aElement->NextSiblingElement())
	{  
		keysPressedPerStep.push_back(findKeyInt(aElement->GetText()));
	}

	// needs to read in the settings 
	aElement = hRoot.FirstChildElement("settings").Element(); 
	pluginName = aElement->Attribute("name");
	recordRate = atof(aElement->Attribute("recordRate"));
	stepRate = atof(aElement->Attribute("stepRate")); 
	
	string trueStr = "true";

	reloadEveryRun = (trueStr.compare(toLower(aElement->Attribute("reloadEveryRun"))) == 0);
	reloadOnChange =  (trueStr.compare(toLower(aElement->Attribute("reloadEveryChange"))) == 0);
	debug = (trueStr.compare(toLower(aElement->Attribute("debug"))) == 0);
	
	// make the vectors of datapoints from the vectors of strings 
	covertToData();
}

/// print the data read from the if needed to XML 
/// used to see what is read in
void debugXML()
{
	cout << "These get reset per run\n";
	for(unsigned int ii = 0; ii < datarefsThatGetResetPerRun.size(); ii++)
		cout << datarefsThatGetResetPerRun[ii]  << "\n";

	cout << "Values they get assigned\n";
	for(unsigned int ii = 0; ii < datarefsThatGetResetPerRunValues.size(); ii++)
		cout << datarefsThatGetResetPerRunValues[ii]  << "\n";

	cout << "These get reset per record\n";
	for(unsigned int ii = 0; ii < datarefsThatGetResetPerRecord.size(); ii++)
		cout << datarefsThatGetResetPerRecord[ii]  << "\n";

	cout << "Values they get assigned\n";
	for(unsigned int ii = 0; ii < datarefsThatGetResetPerRecordValues.size(); ii++)
		cout << datarefsThatGetResetPerRecordValues[ii]  << "\n";

	cout << "These get reset per step\n";
	for(unsigned int ii = 0; ii < datarefsThatGetResetPerStep.size(); ii++)
		cout << datarefsThatGetResetPerStep[ii]  << "\n";

	cout << "Values they get assigned\n";
	for(unsigned int ii = 0; ii < datarefsThatGetResetPerStepValues.size(); ii++)
		cout << datarefsThatGetResetPerStepValues[ii]  << "\n";

	cout << "Things that get measured\n";  
	for(unsigned int ii = 0; ii < datarefsThatGetMeasured.size(); ii++)
		cout << datarefsThatGetMeasured[ii]  << "\n";

	cout << "Things that get changed\n";
	for(unsigned int ii = 0; ii < datarefsThatGetChanged.size(); ii++)
	{
		cout << datarefsThatGetChanged[ii]  << ": ";
		for(unsigned int jj = 0; jj < valuesThatCanBeChosen[ii].size(); jj++)
			cout << valuesThatCanBeChosen[ii][jj]  << " ";
		cout << "\n";
	}

	cout << "Values that have been learned\n";
	for(unsigned int ii = 0; ii < datarefsThatHaveBeenLearned.size(); ii++)
	{    
		cout << datarefsThatHaveBeenLearned[ii]  << ": ";
		for(unsigned int jj = 0; jj < datarefsReadForTheDecisions[ii].size(); jj++)
		{
			cout << datarefsReadForTheDecisions[ii][jj]  << " ";
			cout << weightsForLearnedDatarefs[ii][jj][0]  << " ";
		}
		cout << "These are the choices\n";
		for(unsigned int jj = 0; jj < valuesThatCanBeChosenForLearnedDatarefs[ii].size(); jj++)
			cout << valuesThatCanBeChosenForLearnedDatarefs[ii][jj]  << "\n";
	} 

	cout << "Settings\n";
	cout << pluginName << endl;
	cout << recordRate << endl;
	cout << stepRate << endl;
	cout << reloadEveryRun << endl;
	cout << reloadOnChange << endl;
	cout << debug << endl;
}

// register a callback for every loop so at the step interval the
// plugin can interact with the simulator
static float MyFlightLoopCallback(float inElapsedSinceLastCall, 
		float  inElapsedTimeSinceLastFlightLoop, 
		int inCounter, void *  inRefcon);

// itilizes the plugin 
// first part of the code called by Xplane
PLUGIN_API int XPluginStart(char *	outName, char *	outSig, char *outDesc)
{
	// make the vectors needed to lookup the button and key numbers 
	makecmdButtonArray();
	makecmdKeyArray();
	// read in the data from "config.xml"
	loadXML(xmlFileName.c_str());	

	char    outputPath[255];

	// data for Xplane to read describing what the plugin is
	strcpy(outName, pluginName.c_str());
	strcpy(outSig, "xplanesdk.USNA.MachineLearning");
	strcpy(outDesc, "A plugin that reads config.xml for Machine Learning.");


	XPLMGetSystemPath(outputPath);
	strcat(outputPath, "log.txt");

	// sets the random number generator for making guesses
	srand ( time(NULL) );

	// tell xplane what to start calling and ho often to call it
	XPLMRegisterFlightLoopCallback(MyFlightLoopCallback,stepRate,NULL);

	return 1;
}

/// run when Xplane is turned off
PLUGIN_API void	XPluginStop(void)
{
	// tell Xplane that it can stop looping
	XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);
}

/// need to have it although it does not do any thing
PLUGIN_API void XPluginDisable(void)
{

}

/// just needs to return 1
PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

/// handles messages passed from other processes and threads in Xplane
/// used to catch the crash and reload messages to reset runs
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID	inFromWho, long inMessage, void * inParam)
{
	// 102 and 101 are sent when the plane crashes or starts out
	// used to so Xplane knows when a new run starts
	if(inMessage == 101 || inMessage == 102)
	{
		isFirst = true;
	}
}

/// moves the plane to a given x y and z
/// only works in the local area limited to about 1000m  in every direction
/// very tricky to get right
PLUGIN_API void movePlane(double x,double y,double z)
{
	XPLMSetDataf(XPLMFindDataRef("sim/flightmodel/position/local_x"),x);
	XPLMSetDataf(XPLMFindDataRef("sim/flightmodel/position/local_y"),y);
	XPLMSetDataf(XPLMFindDataRef("sim/flightmodel/position/local_z"),z);
}

/// sets the header for your data so you dont forget what you just measured
void printStartLine()
{
	if(dpMeasured.size()==0)
		return;
	int ii =0;
	for (ii =0; ii< dpMeasured.size(); ii++)
		cout << dpMeasured[ii].name << ", ";

	for (ii =0; ii< dpChanged.size() - 1; ii++)
	{
		cout << dpChanged[ii].name << ", ";
	}

	cout << dpChanged[ii].name << ", " << "\n";
}

/// this is the loop that is called on the step interval
/// this is where all the programs time is spent
float	MyFlightLoopCallback(float inElapsedSinceLastCall, 
			     float inElapsedTimeSinceLastFlightLoop,
			     int inCounter, 
			     void * inRefcon)
{
	// is the first run through the loop
	if(isStart)
	{
		isStart = false;
		printStartLine();
	}
	// isFirst is set by the message handlers
	if(isFirst)
	{
		// take care of all run actions
		// press keys, probably brakes 
		pressKeys(keysPressedPerRun);
		// press buttons
		pressButtons(buttonsPressedPerRun);
		// any Data points that need to be reset
		resetDPs(dpPerRun,  datarefsThatGetResetPerRunValues);

		// will reload the XML every run if set in the intail XML
		// usefull to fix problems with experiment configuration
		// great way to make sure that the XML wrong
		if(debug)
		{
			debugXML();
		}
		// if reload every run is set in the XML then all the simulator varibles will be reset
		if(reloadEveryRun)
		{
			loadXML(xmlFileName.c_str());
		}
		// resets the timefactor so you know time from load of plane on runway for this run
		// not from when the plugin was initialized
		timeFactor = XPLMGetElapsedTime();
		// makes sure that the start of the run code is not run every step
		isFirst = false;
		// makes sure to normalize the altitiude of the plane for teh elevation of 
		// the runway
		runwayElevation = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/elevation"));
		// how many loops the plane has gone through
		xplround = 0;
		int count =0;
	}

	// take care of all step actions
	resetDPs(dpPerStep,  datarefsThatGetResetPerStepValues);
	pressKeys(keysPressedPerStep);
	pressButtons(buttonsPressedPerStep);
	// if we have passed the 
	if(stepsum>recordRate)
	{
		stepsum = 0;
		// take care of all record options
		resetDPs(dpPerRecord,  datarefsThatGetResetPerRecordValues);
		pressKeys(keysPressedPerRecord);
		pressButtons(buttonsPressedPerRecord);
		makeGuesses();
		makeChoices();
	}

	xplround++;
	stepsum += stepRate;
	return stepRate; // tell xplane to call back in <steoRate> seconds
}                                   

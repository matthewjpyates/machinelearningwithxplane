#include <math.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "tinyxml.h"
#include <vector>
#include <stdlib.h>
#include <string>
#include <iostream>

//#include "stdafx.h"
using namespace std;

//file to be read 
string xmlFileName = "config.xml"; 
string pluginName = "Some random plugin name that was not set by the XML";
double recordRate = 1;
double stepRate = 1;
bool reloadEveryRun = true;
bool reloadOnChange = true;
bool debug = false;

// Global Varibles needed that hold the lists of datarefs
// everytime the plane resets on the runway these lists apply these value
vector<string> datarefsThatGetResetPerRun;
vector<double> datarefsThatGetResetPerRunValues;
// values that every record step are reset
vector<string> datarefsThatGetResetPerRecord;
vector<double> datarefsThatGetResetPerRecordValues;
// vales that are reset every step
vector<string> datarefsThatGetResetPerStep;
vector<double> datarefsThatGetResetPerStepValues;
// these are writen to standout every record
vector<string> datarefsThatGetMeasured;
// these are the values that need to be changed every step 
vector<string> datarefsThatGetChanged;
vector<vector<double> > valuesThatCanBeChosen;
// these are the learned datarefs every record
vector<string> datarefsThatHaveBeenLearned;
vector<vector<string> > datarefsReadForTheDecisions; 
vector<vector<double> > weightsForLearnedDatarefs;
vector<vector<double> > valuesThatCanBeChosenForLearnedDatarefs;


// ----------------------------------------------------------------------
// STDOUT dump and indenting utility functions
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

// load the named file and dump its structure to STDOUT
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

string toLower(string strr)
{	
	char str[300];
	string ret;
	strcpy(str,strr.c_str());
	int differ = 'A'-'a';
	char ch;
	int ii = strlen(str);
	for (int i=0; i <ii;i++)                                                           
	{
		strncpy(&ch,str+i,1);
		if (ch>='A' && ch<='Z')
		{
			ch = ch-differ;
			memcpy(str+i,&ch,1);
		}
	}
	ret = str;
	return ret;
}

void loadXML(const char* pFilename)
{
	TiXmlDocument doc(pFilename);
	if (!doc.LoadFile()) return;
	TiXmlHandle hDoc(&doc);
	//dump_to_stdout( &doc );
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
		cout << aElement->Attribute("dataref");
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
		dfe =  TiXmlHandle(aElement).FirstChild("listweights").FirstChild().Element();
		weightsForLearnedDatarefs.push_back(vector<double>());
		for( ; dfe; dfe=dfe->NextSiblingElement())   
		{  
			weightsForLearnedDatarefs.back().push_back(atof(dfe->GetText()));
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
		
 	aElement = hRoot.FirstChildElement("settings").Element();	
	pluginName = aElement->Attribute("name");
	recordRate = atof(aElement->Attribute("recordRate"));
	stepRate = atof(aElement->Attribute("stepRate")); 
	string trueStr = "true";
	reloadEveryRun = (trueStr.compare(toLower(aElement->Attribute("reloadEveryRun"))) == 0);
	reloadOnChange =  (trueStr.compare(toLower(aElement->Attribute("reloadEveryChange"))) == 0);
	debug = (trueStr.compare(toLower(aElement->Attribute("debug"))) == 0);
}

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
			cout << weightsForLearnedDatarefs[ii][jj]  << " ";
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

int main()
{
	loadXML(xmlFileName.c_str());	
	debugXML();
	return 0;
}


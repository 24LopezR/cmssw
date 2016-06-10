
/** PixelForwardTemplateSmearerPlugin.cc
 * ---------------------------------------------------------------------
 * Description:  see PixelForwardTemplateSmearerPlugin.h
 * Authors:  R. Ranieri (CERN), M. Galanti
 * History: Oct 11, 2006 -  initial version
 * 
 * New Pixel Resolution Parameterization 
 * Introduce SiPixelTemplate Object to Assign Pixel Errors
 * by G. Hu
 * ---------------------------------------------------------------------
 */

// SiPixel Gaussian Smearing
#include "FastSimulation/TrackingRecHitProducer/interface/PixelTemplateSmearerBase.h"
#include "FastSimulation/TrackingRecHitProducer/interface/TrackingRecHitAlgorithmFactory.h"
#include "FastSimulation/TrackingRecHitProducer/interface/TrackingRecHitProduct.h"

// Geometry
//#include "Geometry/Records/interface/TrackerDigiGeometryRecord.h"
#include "Geometry/CommonDetUnit/interface/GeomDetUnit.h"
//#include "Geometry/CommonDetUnit/interface/GeomDetType.h"
#include "Geometry/TrackerGeometryBuilder/interface/RectangularPixelTopology.h"
#include "DataFormats/GeometryVector/interface/LocalPoint.h"
#include "DataFormats/GeometryCommonDetAlgo/interface/MeasurementPoint.h"

// Famos
#include "FastSimulation/Utilities/interface/RandomEngineAndDistribution.h"
#include "FastSimulation/Utilities/interface/SimpleHistogramGenerator.h"

// STL

// ROOT
#include <TFile.h>
#include <TH1F.h>
#include <TH2F.h>
//#include <TAxis.h>

//#define FAMOS_DEBUG

const double microntocm = 0.0001;
using namespace std;


class PixelForwardTemplateSmearerPlugin:
    public PixelTemplateSmearerBase
{
    public:
          explicit PixelForwardTemplateSmearerPlugin(
            const std::string& name,
            const edm::ParameterSet& config,
            edm::ConsumesCollector& consumesCollector
          );
          virtual ~PixelForwardTemplateSmearerPlugin();

    private:
        void initializeForward();
};


PixelForwardTemplateSmearerPlugin::PixelForwardTemplateSmearerPlugin( 
    const std::string& name,
    const edm::ParameterSet& config,
    edm::ConsumesCollector& consumesCollector 
):
    PixelTemplateSmearerBase(name,config,consumesCollector)
{

    isForward = true;
    thePixelResolutionFileName1 = config.getParameter<string>( "NewPixelForwardResolutionFile" );
    thePixelResolutionFile1 = new TFile( edm::FileInPath( thePixelResolutionFileName1 ).fullPath().c_str()  ,"READ");
    thePixelResolutionFileName2 = config.getParameter<string>( "NewPixelForwardResolutionFile2" );
    thePixelResolutionFile2 = new TFile( edm::FileInPath( thePixelResolutionFileName2 ).fullPath().c_str()  ,"READ");
    
    probfileName = config.getParameter<string>( "probfileforward" );
    probfile =new TFile( edm::FileInPath( probfileName ).fullPath().c_str()  ,"READ");
    thePixelResolutionMergedXFileName = config.getParameter<string>( "pixelresxmergedforward" );
    thePixelResolutionMergedXFile = new TFile( edm::FileInPath( thePixelResolutionMergedXFileName ).fullPath().c_str()  ,"READ");
    thePixelResolutionMergedYFileName = config.getParameter<string>( "pixelresymergedforward" );
    thePixelResolutionMergedYFile = new TFile( edm::FileInPath( thePixelResolutionMergedYFileName ).fullPath().c_str()  ,"READ");
    initializeForward();
    
    tempId = config.getParameter<int> ( "templateIdForward" );
    
    if( ! SiPixelTemplate::pushfile(tempId, thePixelTemp_) )
    {
        throw cms::Exception("PixelForwardTemplateSmearerPlugin:") <<"SiPixel Forward Template Not Loaded Correctly!";
    }
}


PixelForwardTemplateSmearerPlugin::~PixelForwardTemplateSmearerPlugin()
{
}


void PixelForwardTemplateSmearerPlugin::initializeForward()
{
    rescotAlpha_binMin = 0.1;
    rescotAlpha_binWidth = 0.1;
    rescotAlpha_binN = 4;
    rescotBeta_binMin = 0.;
    rescotBeta_binWidth = 0.15;
    rescotBeta_binN = 4;
    resqbin_binMin = 0;
    resqbin_binWidth = 1;
    resqbin_binN = 4;

    // Initialize the forward histos once and for all, and prepare the random generation
    for ( unsigned cotalphaHistBin=1; cotalphaHistBin<=rescotAlpha_binN; ++cotalphaHistBin)
    {
        for ( unsigned cotbetaHistBin=1; cotbetaHistBin<=rescotBeta_binN; ++cotbetaHistBin)
        {
            for( unsigned qbinBin=1;  qbinBin<=resqbin_binN; ++qbinBin )
            {
                unsigned int edgePixelHistN = cotalphaHistBin*1000 +  cotbetaHistBin*10 +  qbinBin;
                theXHistos[edgePixelHistN] = new SimpleHistogramGenerator((TH1F*)thePixelResolutionFile1->Get(Form("DQMData/clustFPIX/fhx0%u",edgePixelHistN)));
                theYHistos[edgePixelHistN] = new SimpleHistogramGenerator((TH1F*)thePixelResolutionFile1->Get(Form("DQMData/clustFPIX/fhy0%u",edgePixelHistN)));
                
                unsigned int PixelHistN = 10000 + cotbetaHistBin*100 +  cotalphaHistBin*10 +  qbinBin;
                theXHistos[PixelHistN] = new SimpleHistogramGenerator((TH1F*) thePixelResolutionFile2->Get(Form("hx0%u",PixelHistN)));
                theYHistos[PixelHistN] = new SimpleHistogramGenerator((TH1F*) thePixelResolutionFile2->Get(Form("hy0%u",PixelHistN)));
            }
        }
    }

    for ( unsigned cotalphaHistBin=1; cotalphaHistBin<=rescotAlpha_binN; ++cotalphaHistBin)
    {
        for ( unsigned cotbetaHistBin=1; cotbetaHistBin<=rescotBeta_binN; ++cotbetaHistBin)
        {
            unsigned int SingleBigPixelHistN = 100000 + cotalphaHistBin*100 + cotbetaHistBin;
            theXHistos[SingleBigPixelHistN] = new SimpleHistogramGenerator((TH1F*)thePixelResolutionFile1->Get(Form("DQMData/clustFPIX/fhx%u",SingleBigPixelHistN)));
            theYHistos[SingleBigPixelHistN] = new SimpleHistogramGenerator((TH1F*)thePixelResolutionFile1->Get(Form("DQMData/clustFPIX/fhy%u",SingleBigPixelHistN)));
            
            unsigned int SinglePixelHistN = cotbetaHistBin*10 + cotalphaHistBin;
            theXHistos[SinglePixelHistN]  = new SimpleHistogramGenerator((TH1F*)thePixelResolutionFile2->Get(Form("hx000%u",SinglePixelHistN)));
            theYHistos[SinglePixelHistN]  = new SimpleHistogramGenerator((TH1F*)thePixelResolutionFile2->Get(Form("hy000%u",SinglePixelHistN)));
        }
    }
}

DEFINE_EDM_PLUGIN(
    TrackingRecHitAlgorithmFactory,
    PixelForwardTemplateSmearerPlugin,
    "PixelForwardTemplateSmearerPlugin"
);

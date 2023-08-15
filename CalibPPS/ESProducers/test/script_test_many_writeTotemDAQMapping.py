import FWCore.ParameterSet.Config as cms
import re
import subprocess

# Script which reads data from DB/XML and writes them to file
# Run with python3

# The 'outputFileName' parameter is the name of file to which data will be dropped

diamonds = {
  "dbConnect": "sqlite_file:CTPPSDiamondsScript_DAQMapping.db",
  "outputFileName": "all_diamonds",
  "subSystemName" : "TimingDiamond",
  "multipleChannelsPerPayload": False,
  "configuration": [
    # 2016, before diamonds inserted in DAQ
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("1:min - 283819:max"),
      "mappingFileNames": cms.vstring(),
      "maskFileNames" : cms.vstring()
    },
    # 2016, after diamonds inserted in DAQ
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("283820:min - 292520:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_timing_diamond.xml"),
      "maskFileNames" : cms.vstring()
    },
    # 2017
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("292521:min - 310000:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_timing_diamond_2017.xml"),
      "maskFileNames" : cms.vstring()
    },
    # 2018
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("310001:min - 339999:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_timing_diamond_2018.xml"),
      "maskFileNames" : cms.vstring()
    },
    # 2022
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("340000:min - 362919:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_timing_diamond_2022.xml"),
      "maskFileNames" : cms.vstring()
    },
    # 2023
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("362920:min - 999999999:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_timing_diamond_2023.xml"),
      "maskFileNames" : cms.vstring()
    }
  ]
}

strips = {
    "dbConnect": "sqlite_file:CTPPSStrip_DAQMapping.db",
    "outputFileName": "all_strips",
    "subSystemName": "TrackingStrip",
    "multipleChannelsPerPayload": False,
    "configuration": [
        # 2016, before TS2
        {
          "sampicSubDetId": cms.uint32(6),
          "validityRange" : cms.EventRange("1:min - 280385:max"),
          "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_tracking_strip_2016_to_fill_5288.xml"),
          "maskFileNames" : cms.vstring()
        },
        # 2016, during TS2
        {
          "sampicSubDetId": cms.uint32(6),
          "validityRange" : cms.EventRange("280386:min - 281600:max"),
          "mappingFileNames": cms.vstring(),
          "maskFileNames" : cms.vstring()
        },
        # 2016, after TS2
        {
          "sampicSubDetId": cms.uint32(6),
          "validityRange" : cms.EventRange("281601:min - 290872:max"),
          "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_tracking_strip_2016_from_fill_5330.xml"),
          "maskFileNames" : cms.vstring()
        },
        # 2017
        {
          "sampicSubDetId": cms.uint32(6),
          "validityRange" : cms.EventRange("290873:min - 311625:max"),
          "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_tracking_strip_2017.xml"),
          "maskFileNames" : cms.vstring()
        },
        # 2018
        {
          "sampicSubDetId": cms.uint32(6),
          "validityRange" : cms.EventRange("311626:min - 339999:max"),
          "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_tracking_strip_2018.xml"),
          "maskFileNames" : cms.vstring()
        },
        # 2022
        {
          "sampicSubDetId": cms.uint32(6),
          "validityRange" : cms.EventRange("340000:min - 999999999:max"),
          "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_tracking_strip_2022.xml"),
          "maskFileNames" : cms.vstring()
        }
    ]
}

totemTiming = {
  "dbConnect": "sqlite_file:CTPPSTotemTiming_DAQMapping.db",
  "subSystemName": "TotemTiming",
  "outputFileName": "all_timing",
  "multipleChannelsPerPayload": False,
  "configuration": [
    # 2017, before detector inserted in DAQ
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("1:min - 310000:max"),
      "mappingFileNames": cms.vstring(),
      "maskFileNames" : cms.vstring()
    },
    # 2018
    {
      "sampicSubDetId": cms.uint32(6),
      "validityRange" : cms.EventRange("310001:min - 339999:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_totem_timing_2018.xml"),
      "maskFileNames" : cms.vstring()
    },
    # 2022
    {
      "sampicSubDetId": cms.uint32(5),
      "validityRange" : cms.EventRange("340000:min - 999999999:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_totem_timing_2022.xml"),
      "maskFileNames" : cms.vstring()
    }
  ]
}

t2 = {
  "dbConnect": "sqlite_file:CTPPST2_DAQMapping.db",
  "subSystemName": "TotemT2",
  "outputFileName": "all_t2",
  "multipleChannelsPerPayload": True,
  "configuration": [
    {
      "sampicSubDetId": cms.uint32(7),
      "validityRange" : cms.EventRange("368023:min - 999999999:max"),
      "mappingFileNames": cms.vstring("CondFormats/PPSObjects/xml/mapping_totem_nt2_2023_final.xml"),
      "maskFileNames" : cms.vstring()
    }
  ]
}

#-----------------------------------------------------------------

filesToRead = [totemTiming, diamonds, strips, t2]  
fromDb = True

# For each file change the variable values in the config so that they match the selected XML file and then run the config
for fileContent in filesToRead:
    for fileInfo in fileContent["configuration"]:
        with open('CalibPPS/ESProducers/test/test_writeTotemDAQMapping.py', 'r+') as f:        
            content = f.read()
            # replace values specific for selected detector
            content = re.sub(r'subSystemName =.*', f'subSystemName = "{fileContent["subSystemName"]}"', content)
            content = re.sub(r'process.CondDB.connect =.*', f'process.CondDB.connect = "{fileContent["dbConnect"]}"', content)
            content = re.sub(r'process.totemDAQMappingESSourceXML.multipleChannelsPerPayload =.*', 
                             f'process.totemDAQMappingESSourceXML.multipleChannelsPerPayload = {fileContent["multipleChannelsPerPayload"]}', 
                             content)
            content = re.sub(r'process.totemDAQMappingESSourceXML.sampicSubDetId =.*', 
                             f'process.totemDAQMappingESSourceXML.sampicSubDetId = {fileInfo["sampicSubDetId"]}',
                             content)
            
            
            # replace name of output file and add adequate suffix to it ('_db' or '_xml')
            fileNameExt = fileContent["outputFileName"] + "_db.txt" if fromDb else fileContent["outputFileName"]+"_xml.txt"
            content = re.sub(r'fileName =.*', f'fileName = cms.untracked.string("{fileNameExt}")' , content)
          
            # replace values specific for selected files
            content = re.sub(r'minIov =.*', f'minIov = {fileInfo["validityRange"].start()}', content)
            content = re.sub(r'maxIov =.*', f'maxIov = {fileInfo["validityRange"].end()}', content)
            content = re.sub(r'mappingFileNames =.*', f'mappingFileNames = {fileInfo["mappingFileNames"]},', content)
            content = re.sub(r'maskFileNames =.*', f'maskFileNames = {fileInfo["maskFileNames"]},', content)
            
            # replace values in ESPrefer to read data from DB or from XML
            if fromDb:
              replacement = f'process.es_prefer_totemTimingMapping = cms.ESPrefer("PoolDBESSource", "", \
                TotemReadoutRcd=cms.vstring(f"TotemDAQMapping/{fileContent["subSystemName"]}"))'
            else:          
              replacement = f'process.es_prefer_totemTimingMapping = cms.ESPrefer("TotemDAQMappingESSourceXML", \
                "totemDAQMappingESSourceXML", TotemReadoutRcd=cms.vstring("TotemDAQMapping/{fileContent["subSystemName"]}"))'
                
            content = re.sub(r'process.es_prefer_totemTimingMapping =.*', replacement, content)            
          
            f.seek(0)
            f.write(content)
            f.truncate()
            
            
        subprocess.run(f'cmsRun CalibPPS/ESProducers/test/test_writeTotemDAQMapping.py' , shell=True)
    

package be.uantwerpen.fti.ds.smartracecar.common;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.HashMap;

public class XMLUtils {

    //Load all current available offline maps from the /mapFolder folder. It reads and maps.xml file with all the necessary information.
    public static HashMap<String,Map> loadMaps(String mapFolder) {
        HashMap<String,Map> loadedMaps = new HashMap<>();

        try {
            File fXmlFile = new File(mapFolder + "/maps.xml");
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(fXmlFile);
            doc.getDocumentElement().normalize();

            NodeList nList = doc.getElementsByTagName("map");

            for (int temp = 0; temp < nList.getLength(); temp++) {

                Node nNode = nList.item(temp);

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element eElement = (Element) nNode;
                    String name = eElement.getElementsByTagName("name").item(0).getTextContent();
                    float meterPerPixel = Float.parseFloat(eElement.getElementsByTagName("meterPerPixel").item(0).getTextContent());
                    loadedMaps.put(name,new Map(name, meterPerPixel));
                    Log.logConfig("CORE","Added map: " + name + ".");
                }
            }
        } catch (Exception e) {
            Log.logSevere("CORE","Could not correctly load XML of maps." + e);
        }
        return loadedMaps;
    }


}

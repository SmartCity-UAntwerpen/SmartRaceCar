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
                    loadedMaps.put(name,new Map(name));
                    Log.logConfig("XML","Added map: " + name + ".");
                }
            }
        } catch (Exception e) {
            Log.logSevere("XML","Could not correctly load XML of maps." + e);
        }
        return loadedMaps;
    }

    public static HashMap<Integer,WayPoint> loadWaypoints(String mapFolder) {
        HashMap<Integer,WayPoint> loadedMaps = new HashMap<>();

        try {
            File fXmlFile = new File(mapFolder + "/waypoints.xml");
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(fXmlFile);
            doc.getDocumentElement().normalize();

            NodeList nList = doc.getElementsByTagName("waypoint");

            for (int temp = 0; temp < nList.getLength(); temp++) {

                Node nNode = nList.item(temp);

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element eElement = (Element) nNode;
                    int id  = Integer.parseInt(eElement.getElementsByTagName("id").item(0).getTextContent());
                    float x = Float.parseFloat(eElement.getElementsByTagName("x").item(0).getTextContent());
                    float y = Float.parseFloat(eElement.getElementsByTagName("y").item(0).getTextContent());
                    float z = Float.parseFloat(eElement.getElementsByTagName("z").item(0).getTextContent());
                    float w = Float.parseFloat(eElement.getElementsByTagName("w").item(0).getTextContent());
                    int weight  = Integer.parseInt(eElement.getElementsByTagName("weight").item(0).getTextContent());
                    loadedMaps.put(id,new WayPoint(id,x,y,z,w,weight));
                    Log.logConfig("XML","Added wayPoint: " + id + " and coordinates " + x +"," + y +"," + z +"," + w +" and weight " + weight + ".");
                }
            }
        } catch (Exception e) {
            Log.logSevere("XML","Could not correctly load XML of waypoints." + e);
        }
        return loadedMaps;
    }


}

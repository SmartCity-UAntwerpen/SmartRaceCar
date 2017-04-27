package SmartRacecar;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.*;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class Map {

    private String name = "";
    private Position startPosition = new Position(0, 0);
    private float meterPerPixel = 0;

    public Map(String name, Position startPosition, float meterPerPixel) {
        this.name = name;
        this.startPosition = startPosition;
        this.meterPerPixel = meterPerPixel;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Position getStartPosition() {
        return startPosition;
    }

    public void setStartPosition(Position startPosition) {
        this.startPosition = startPosition;
    }

    public float getMeterPerPixel() {
        return meterPerPixel;
    }

    public void setMeterPerPixel(float meterPerPixel) {
        this.meterPerPixel = meterPerPixel;
    }

    public void setStartPosition(int x, int y) {
        this.startPosition.setX(x);
        this.startPosition.setY(y);
    }

    public static ArrayList<Map> loadMapsFromFolder(String path) {
        ArrayList<Map> maps = new ArrayList<>();

        try {

            File fXmlFile = new File("maps/maps.xml");
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(fXmlFile);

            //optional, but recommended
            //read this - http://stackoverflow.com/questions/13786607/normalization-in-dom-parsing-with-java-how-does-it-work
            doc.getDocumentElement().normalize();

            NodeList nList = doc.getElementsByTagName("map");

            for (int temp = 0; temp < nList.getLength(); temp++) {

                Node nNode = nList.item(temp);

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element eElement = (Element) nNode;
                    String name = eElement.getElementsByTagName("name").item(0).getTextContent();
                    float x = Float.parseFloat(eElement.getElementsByTagName("startPositionX").item(0).getTextContent());
                    float y = Float.parseFloat(eElement.getElementsByTagName("startPositionY").item(0).getTextContent());
                    float meterPerPixel = Float.parseFloat(eElement.getElementsByTagName("meterPerPixel").item(0).getTextContent());
                    maps.add(new Map(name, new Position(x, y), meterPerPixel));
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return maps;
    }

    private static String getFileExtension(File file) {
        String name = file.getName();
        try {
            return name.substring(name.lastIndexOf(".") + 1);
        } catch (Exception e) {
            return "";
        }
    }

    public static Map addMap(String name, float x, float y, float meterPerPixel) {
        Map map = new Map(name, new Position(x, y), meterPerPixel);
        try {
            DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder documentBuilder = null;

            documentBuilder = documentBuilderFactory.newDocumentBuilder();

            Document document = documentBuilder.parse("maps/maps.xml");
            Element root = document.getDocumentElement();

                Element newMap = document.createElement("map");

                Element newName = document.createElement("name");
                newName.appendChild(document.createTextNode(name));
                newMap.appendChild(newName);

                Element newPositionX = document.createElement("startPositionX");
                newPositionX.appendChild(document.createTextNode(Float.toString(x)));
                newMap.appendChild(newPositionX);

                Element newPositionY = document.createElement("startPositionY");
                newPositionY.appendChild(document.createTextNode(Float.toString(y)));
                newMap.appendChild(newPositionY);

                Element newMeterPerPixel = document.createElement("meterPerPixel");
                newMeterPerPixel.appendChild(document.createTextNode(Float.toString(meterPerPixel)));
                newMap.appendChild(newMeterPerPixel);

                root.appendChild(newMap);

                DOMSource source = new DOMSource(document);

                TransformerFactory transformerFactory = TransformerFactory.newInstance();
                Transformer transformer = transformerFactory.newTransformer();
                transformer.setOutputProperty(OutputKeys.INDENT, "yes");
                transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
                StreamResult result = new StreamResult("maps/maps.xml");
                transformer.transform(source, result);

        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (TransformerConfigurationException e) {
            e.printStackTrace();
        } catch (TransformerException e) {
            e.printStackTrace();
        }
        return map;

    }
}

package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.core.Communication.MapBackendCommunicator;
import be.uantwerpen.fti.ds.sc.core.Communication.MapVehicleCommunicator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;

public class MapManager implements MQTTListener
{
	private CoreParameters params;
	private MapBackendCommunicator backend;
	private MapVehicleCommunicator vehicle;
	private MQTTUtils mqttUtils;

	private HashMap<String, Map> loadedMaps;                                // Map of all loaded maps.

	private Logger log;

	public MapManager(Core core, CoreParameters params, MapBackendCommunicator backend, MapVehicleCommunicator vehicle)
	{
		this.log = LoggerFactory.getLogger(MapManager.class);
		this.params = params;

		this.backend = backend;
		this.vehicle = vehicle;

		this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic("racecar/changeMap/" + core.getID());

		this.log.info("starting to load maps");
		this.loadedMaps = new HashMap<>();
		String mapsFolder = this.findMapsFolder();
		this.log.info("Found maps folder = " + mapsFolder);
		this.loadedMaps = this.loadMaps(mapsFolder);
		this.log.info("Maps loaded");
	}

	/**
	 * Load all current available offline maps from the /mapFolder folder.
	 * It reads the maps.xml file with all the necessary information.
	 *
	 * @param mapFolder location of the maps.xml file.
	 * @return Returns a Hashmap<String,Map> where the String is the mapname. It contains all loaded maps.
	 */
	private HashMap<String, Map> loadMaps(String mapFolder)
	{
		HashMap<String, Map> loadedMaps = new HashMap<>();
		try
		{
			File fXmlFile = new File(mapFolder + "/maps.xml");
			DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
			Document doc = dBuilder.parse(fXmlFile);
			doc.getDocumentElement().normalize();

			NodeList nList = doc.getElementsByTagName("map");

			for (int temp = 0; temp < nList.getLength(); temp++)
			{

				Node nNode = nList.item(temp);

				if (nNode.getNodeType() == Node.ELEMENT_NODE)
				{

					Element eElement = (Element) nNode;
					String name = eElement.getElementsByTagName("name").item(0).getTextContent();
					loadedMaps.put(name, new Map(name));
					this.log.info("Added map: " + name + ".");
				}
			}
		} catch (IOException | SAXException | ParserConfigurationException e)
		{
			this.log.error("Could not correctly load XML of maps." + e);
		}
		return loadedMaps;
	}

	/**
	 * Find the location of the maps.xml containing folder. Searches up to 3 levels deep.
	 *
	 * @return returns a String containing the location of the maps.xml's absolute path.
	 */
	private String findMapsFolder()
	{
		this.log.info("starting to search map folder");
		FileUtils fileUtils = new FileUtils();
		fileUtils.searchDirectory(new File("."), "maps.xml");
		this.log.info("search complete");
		this.log.info("Found " + fileUtils.getResult().size() + " matches for maps.xml");

		if(fileUtils.getResult().size() == 0)
		{
			this.log.debug("could not find maps.xml");
			System.exit(0);
		}

		String output = null;
		for (String matched : fileUtils.getResult())
		{
			output = matched;
		}

		this.log.info("Maps folder found at " + output);

		return output;
	}

	/**
	 * REST GET request to RacecarBackend to request the name of the current map. If this map is not found in the offline available maps, it does another
	 * REST GET request to download the map files and store it in the mapfolder and add it to the maps.xml file.
	 * After that it sends this information to the vehicle SimKernel/SimKernel over the socket connection.
	 */
	public boolean configureMap()
	{
		boolean contains;


		String mapName = this.backend.getMapName();
		if (this.loadedMaps.containsKey(mapName))
		{
			contains = true;
			this.log.info("Current used map '" + mapName + "' found in folder, setting as current map.");

		}
		else
		{
			contains = false;
			this.backend.downloadMap(mapName);


			Map map = new Map(mapName);
			this.loadedMaps.put(mapName, map);
			this.log.info("Added downloaded map : " + mapName);

			File mapDir = new File(this.findMapsFolder());
			String mapPath = mapDir + "/maps.xml";
			this.writeMapsXML(mapPath, mapName);

			this.log.info("Current map '" + mapName + "' downloaded and set as current map.");
		}

		if (!this.params.isDebug())
		{
			this.vehicle.setMap(this.loadedMaps.get(mapName));
		}

		return contains;
	}

	/**
	 * Writes new map info to maps.xml file.
	 * @param mapPath path to maps.xml file
	 * @param mapName name of new map
	 */
	private void writeMapsXML(String mapPath, String mapName)
	{
		try
		{
			DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

			Document document = documentBuilder.parse(mapPath);
			Element root = document.getDocumentElement();

			Element newMap = document.createElement("map");

			Element newName = document.createElement("name");
			newName.appendChild(document.createTextNode(mapName));
			newMap.appendChild(newName);

			root.appendChild(newMap);

			TransformerFactory transformerFactory = TransformerFactory.newInstance();
			Transformer transformer = transformerFactory.newTransformer();
			transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");
			transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "4");
			DOMSource source = new DOMSource(document);

			StreamResult result = new StreamResult(mapPath);

			transformer.transform(source, result);

		}
		catch (ParserConfigurationException | SAXException | IOException | TransformerException e)
		{
			e.printStackTrace();
			this.log.warn("Could not add map to XML of maps." + e);
		}
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		this.configureMap();
	}
}

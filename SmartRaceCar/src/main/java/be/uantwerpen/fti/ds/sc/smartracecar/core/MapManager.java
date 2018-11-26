package be.uantwerpen.fti.ds.sc.smartracecar.core;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
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

public class MapManager
{
	private Core core;

	private HashMap<String, Map> loadedMaps; 								// Map of all loaded maps.

	private LogbackWrapper log;

	public MapManager(Core core)
	{
		this.core = core;

		this.log = new LogbackWrapper();

		this.loadedMaps = new HashMap<>();
		this.loadedMaps = loadMaps(findMapsFolder());
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
			File fXmlFile = new File(mapFolder);
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
					this.log.info("MAP-MANAGER", "Added map: " + name + ".");
				}
			}
		}
		catch (IOException | SAXException | ParserConfigurationException e)
		{
			this.log.error("MAP-MANAGER", "Could not correctly load XML of maps." + e);
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
		FileUtils fileUtils = new FileUtils();
		fileUtils.searchDirectory(new File(".."), "maps.xml");
		if (fileUtils.getResult().size() == 0)
		{
			fileUtils.searchDirectory(new File("./.."), "maps.xml");
			if (fileUtils.getResult().size() == 0)
			{
				fileUtils.searchDirectory(new File("./../.."), "maps.xml");
				if (fileUtils.getResult().size() == 0)
				{
					fileUtils.searchDirectory(new File("./../../.."), "maps.xml");
					if (fileUtils.getResult().size() == 0)
					{
						this.log.error("CORE", "maps.xml not found. Make sure it exists in some folder (maximum 3 levels deep).");
						System.exit(0);
					}
				}
			}
		}
		String output = null;
		for (String matched : fileUtils.getResult())
		{
			output = matched;
		}
		return output;
	}

	/**
	 * REST GET request to RacecarBackend to request the name of the current map. If this map is not found in the offline available maps, it does another
	 * REST GET request to download the map files and store it in the mapfolder and add it to the maps.xml file.
	 * After that it sends this information to the vehicle SimKernel/SimKernel over the socket connection.
	 */
	public void requestMap()
	{
		String mapName = this.core.getRestUtils().getTextPlain("getmapname");
		if (this.loadedMaps.containsKey(mapName))
		{
			this.log.info("MAP-MANAGER", "Current used map '" + mapName + "' found in folder, setting as current map.");
			if (!this.core.getParams().isDebug())
				this.core.getTcpUtils().sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentMap", this.loadedMaps.get(mapName)));
		}
		else
		{
			Log.logConfig("MAP-MANAGER", "Current used map '" + mapName + "' not found. Downloading...");
			this.core.getRestUtils().getFile("getmappgm/" + mapName, findMapsFolder(), mapName, "pgm");
			this.core.getRestUtils().getFile("getmapyaml/" + mapName, findMapsFolder(), mapName, "yaml");
			Map map = new Map(mapName);
			try
			{
				DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
				DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

				Document document = documentBuilder.parse(findMapsFolder() + "/maps.xml");
				Element root = document.getDocumentElement();

				Element newMap = document.createElement("map");

				Element newName = document.createElement("name");
				newName.appendChild(document.createTextNode(mapName));
				newMap.appendChild(newName);

				root.appendChild(newMap);

				DOMSource source = new DOMSource(document);

				TransformerFactory transformerFactory = TransformerFactory.newInstance();
				Transformer transformer = transformerFactory.newTransformer();
				transformer.setOutputProperty(OutputKeys.INDENT, "yes");
				transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
				StreamResult result = new StreamResult(findMapsFolder() + "/maps.xml");
				transformer.transform(source, result);

			}
			catch (ParserConfigurationException | SAXException | IOException | TransformerException e)
			{
				this.log.warning("MAP-MANAGER", "Could not add map to XML of maps." + e);
			}
			this.loadedMaps.put(mapName, map);
			this.log.info("MAP-MANAGER", "Added downloaded map : " + mapName + ".");
			this.log.info("MAP-MANAGER", "Current map '" + mapName + "' downloaded and set as current map.");
			if (!this.core.getParams().isDebug())
				this.core.getTcpUtils().sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentMap", this.loadedMaps.get(mapName)));
		}
	}
}

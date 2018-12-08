package be.uantwerpen.fti.ds.sc.smartracecar.common;

import com.google.gson.*;

import java.lang.reflect.Type;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Help functions to deal with JSON messages. Uses Google GSON library.
 */
public class JSONUtils
{

	private static final Gson gson = new Gson(); // GSON object to use the library functions.

	/**
	 * Method to verify if string is a valid JSON string.
	 *
	 * @param jsonInString String to be verified.
	 * @return boolean to state if the string is valid JSON. True means valid.
	 */
	public static boolean isJSONValid(String jsonInString)
	{
		try
		{
			gson.fromJson(jsonInString, Object.class);
			return true;
		} catch (com.google.gson.JsonSyntaxException ex)
		{
			LogbackWrapper log = new LogbackWrapper(JSONUtils.class);
			log.warning("JSON", "Not a valid JSON string." + jsonInString + "." + ex);
			return false;
		}
	}

	/**
	 * Get the first word of the JSON object. In this case this is always the keyword used in the message.
	 *
	 * @param jsonInString String to be searched for the keyword.
	 * @return String that the holds the found keyword.
	 */
	public static String getFirst(String jsonInString)
	{
		JsonParser parser = new JsonParser();
		JsonElement element = parser.parse(jsonInString);
		JsonObject object = element.getAsJsonObject();
		Set<Map.Entry<String, JsonElement>> entries = object.entrySet();
		for (Map.Entry<String, JsonElement> entry : entries)
		{
			return entry.getKey();
		}
		return jsonInString;
	}

	/**
	 * Get the object after the keyword of the JSON object. This is always the package of the message.
	 * Converts it to the correct requested Java Object type.
	 *
	 * @param jsonInString String to be searched for the object.
	 * @param t            Type of the object being searched.
	 * @return Object that was requested
	 */
	public static Object getObjectWithKeyWord(String jsonInString, Type t)
	{
		JsonParser parser = new JsonParser();
		JsonElement element = parser.parse(jsonInString);
		JsonObject object = element.getAsJsonObject();
		Set<Map.Entry<String, JsonElement>> entries = object.entrySet();
		for (Map.Entry<String, JsonElement> entry : entries)
		{
			return gson.fromJson(entry.getValue(), t);
		}
		return null;
	}

	/**
	 * Get the object held in the JSON object.
	 * Converts it to the correct requested Java Object type.
	 *
	 * @param jsonInString String to be searched for the object.
	 * @param t            Type of the object being searched.
	 * @return Object that was requested
	 */
	public static Object getObject(String jsonInString, Type t)
	{
		return gson.fromJson(jsonInString, t);
	}

	/**
	 * Converts a Java object of a specific type to a JSON string with a keyword as first element.
	 *
	 * @param keyword Keyword to be used at start of the JSON string to identify the string.
	 * @param object  Object to be parsed.
	 * @return Coverted JSON string containing the keyword and the converted object.
	 */
	public static String objectToJSONStringWithKeyWord(String keyword, Object object)
	{
		JsonObject jsonObject = new JsonObject();
		jsonObject.add(keyword, gson.toJsonTree(object));
		return jsonObject.toString();
	}

	/**
	 * Converts a Java list of a specific type to a JSON string with a keyword as first element.
	 *
	 * @param keyword   Keyword to be used at start of the JSON string to identify the string.
	 * @param arrayList List to be parsed.
	 * @return Coverted JSON string containing the keyword and the converted list.
	 */
	public static String arrayToJSONStringWithKeyWord(String keyword, List<?> arrayList)
	{
		JsonObject jsonObject = new JsonObject();
		jsonObject.add(keyword, gson.toJsonTree(arrayList));
		return jsonObject.toString();
	}

	/**
	 * Converts a Java object of a specific type to a JSON string.
	 *
	 * @param object Object to be parsed.
	 * @return Coverted JSON string containing the converted object.
	 */
	public static String objectToJSONString(Object object)
	{
		return gson.toJson(object);
	}

	/**
	 * Converts a Java list object of a specific type to a JSON string.
	 *
	 * @param arrayList List to be parsed.
	 * @return Coverted JSON string containing the converted list.
	 */
	public static String arrayToJSONString(List<?> arrayList)
	{
		return gson.toJson(arrayList);
	}

	/**
	 * Converts only a keyword to a JSON string.
	 *
	 * @param keyword to be parsed.
	 * @return Converted keyword to JSON String.
	 */
	public static String keywordToJSONString(String keyword)
	{
		JsonObject parentData = new JsonObject();
		JsonObject childData = new JsonObject();
		parentData.add(keyword, childData);
		return parentData.toString();
	}
}
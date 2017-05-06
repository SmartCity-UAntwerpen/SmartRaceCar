package SmartRacecar;

import com.google.gson.*;
import java.lang.reflect.Type;
import java.util.Map;
import java.util.Set;

//Help functions to deal with JSON messages. Uses Google GSON library.
class JSONUtils {

    private static final Gson gson = new Gson();

    JSONUtils(){
    }

    //Method to verify if string is a valid JSON string.
    static boolean isJSONValid(String jsonInString) {
        try {
            gson.fromJson(jsonInString, Object.class);
            return true;
        } catch(com.google.gson.JsonSyntaxException ex) {
            Core.logWarning("JSON","Not a valid JSON string." + jsonInString + "." + ex);
            return false;
        }
    }

    //Get the first word of the JSON object. In this case this is always the keyword used in the message.
    static String getFirst(String jsonInString) {
        JsonParser parser = new JsonParser();
        JsonElement element = parser.parse(jsonInString);
        JsonObject object = element.getAsJsonObject();
        Set<Map.Entry<String, JsonElement>> entries = object.entrySet();
        for (Map.Entry<String, JsonElement> entry : entries) {
            return entry.getKey();
        }
        return jsonInString;
    }

    //Get the object after the keyword of the JSON object. This is always the package of the message.
    //Converts it to the correct requested Java Object type.
    static Object getObject(String jsonInString,Type t){
        JsonParser parser = new JsonParser();
        JsonElement element = parser.parse(jsonInString);
        JsonObject object = element.getAsJsonObject();
        Set<Map.Entry<String, JsonElement>> entries = object.entrySet();
        for (Map.Entry<String, JsonElement> entry : entries) {
            return gson.fromJson(entry.getValue(), t);
        }
        return null;
    }

    //Converts a Java object of a specific type to a JSON string with a keyword as first element.
    static String objectToJSONString(String keyword,Object object){
        JsonObject jsonObject = new JsonObject();
        jsonObject.add(keyword,gson.toJsonTree(object));
        return jsonObject.toString();
    }

    //Converts only a keyword to a JSON string.
    static String keywordToJSONString(String keyword){
        JsonObject parentData = new JsonObject();
        JsonObject childData = new JsonObject();
        parentData.add(keyword, childData);
        return parentData.toString();
    }
}
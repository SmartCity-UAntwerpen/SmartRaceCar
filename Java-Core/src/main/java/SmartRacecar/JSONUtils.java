package SmartRacecar;

import com.google.gson.*;
import java.lang.reflect.Type;
import java.util.Map;
import java.util.Set;

class JSONUtils {

    private static final Gson gson = new Gson();

    JSONUtils(){
    }

    static boolean isJSONValid(String jsonInString) {
        try {
            gson.fromJson(jsonInString, Object.class);
            return true;
        } catch(com.google.gson.JsonSyntaxException ex) {
            Core.logWarning("JSON","Not a valid JSON string." + jsonInString + "." + ex);
            return false;
        }
    }

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

    static String objectToJSONString(String keyword,Object object){
        JsonObject jsonObject = new JsonObject();
        jsonObject.add(keyword,gson.toJsonTree(object));
        Core.logConfig("JSON",gson.toJson(object));
        return jsonObject.toString();
    }

    static String keywordToJSONString(String keyword){
        JsonObject parentData = new JsonObject();
        JsonObject childData = new JsonObject();
        parentData.add(keyword, childData);
        return parentData.toString();
    }
}
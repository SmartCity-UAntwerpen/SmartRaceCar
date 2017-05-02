package SmartRacecar;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.json.simple.JSONObject;

import java.util.Map;
import java.util.Set;

class JSONUtils {

    private eventListener listener;
    private static final Gson gson = new Gson();

    JSONUtils(eventListener listener){
        this.listener = listener;
    }

    boolean isJSONValid(String jsonInString) {
        try {
            gson.fromJson(jsonInString, Object.class);
            return true;
        } catch(com.google.gson.JsonSyntaxException ex) {
            listener.logWarning("JSON","Not a valid JSON string." + jsonInString + "." + ex);
            return false;
        }
    }

    String JSONtoString(JSONObject json){
        return json.toString();
    }

    String getFirst(String jsonInString) {
        JsonParser parser = new JsonParser();
        JsonElement element = parser.parse(jsonInString);
        JsonObject object = element.getAsJsonObject();
        Set<Map.Entry<String, JsonElement>> entries = object.entrySet();
        for (Map.Entry<String, JsonElement> entry : entries) {
            return entry.getKey();
        }
        return jsonInString;
    }
}
package SmartRacecar;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.util.Map;
import java.util.Set;

public final class JSONUtils {
    private static final Gson gson = new Gson();

    private JSONUtils(){}

    public static boolean isJSONValid(String jsonInString) {
        try {
            gson.fromJson(jsonInString, Object.class);
            return true;
        } catch(com.google.gson.JsonSyntaxException ex) {
            System.err.println("[JSON] [ERROR] Not a valid JSON string." + jsonInString + "." + ex);
            return false;
        }
    }

    public static String getFirst(String jsonInString) {
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
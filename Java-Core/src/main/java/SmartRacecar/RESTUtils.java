package SmartRacecar;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;

public class RESTUtils {

    static final String REST_URL = "http://localhost:8080/x";

    public void putJSON(String urlEnd,String json) {
        try {
            URL url = null;
            url = new URL(REST_URL + urlEnd);
            URLConnection connection = null;
            connection = url.openConnection();

            connection.setDoOutput(true);
            connection.setRequestProperty("Content-Type", "application/json");
            connection.setConnectTimeout(5000);
            connection.setReadTimeout(5000);

            OutputStreamWriter out = null;
            out = new OutputStreamWriter(connection.getOutputStream());
            out.write(json);
            out.close();
        } catch (MalformedURLException e) {
            System.err.println("[REST] [ERROR] Not a valid URL." + e);
        } catch (IOException e) {
            System.err.println("[REST] [ERROR] Could not send JSON." + e);
        }
    }
}

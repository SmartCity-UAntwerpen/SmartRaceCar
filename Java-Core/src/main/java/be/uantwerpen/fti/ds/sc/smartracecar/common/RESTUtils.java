package be.uantwerpen.fti.ds.sc.smartracecar.common;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;

//All REST function calls
public class RESTUtils {

    private String REST_URL;

    public RESTUtils(String REST_URL){
        this.REST_URL = REST_URL;
    }


    void putJSON(String urlEnd,String json) {
        try {
            URL url = new URL(REST_URL + urlEnd);
            URLConnection connection = url.openConnection();
            connection.setDoOutput(true);
            connection.setRequestProperty("Content-Type", "application/json");
            connection.setConnectTimeout(5000);
            connection.setReadTimeout(5000);

            OutputStreamWriter out = new OutputStreamWriter(connection.getOutputStream());
            out.write(json);
            out.close();
        } catch (MalformedURLException e) {
            Log.logSevere("REST","Not a valid URL." + e);
            System.err.println("[REST] [ERROR] Not a valid URL." + e);
        } catch (IOException e) {
            Log.logSevere("REST","Could not send JSON." + e);
        }
    }
}

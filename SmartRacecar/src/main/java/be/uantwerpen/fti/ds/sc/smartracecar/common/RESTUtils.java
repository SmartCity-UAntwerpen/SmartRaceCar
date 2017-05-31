package be.uantwerpen.fti.ds.sc.smartracecar.common;

import javax.ws.rs.*;
import javax.ws.rs.client.*;
import javax.ws.rs.core.Response;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;

public class RESTUtils {

    private WebTarget webTarget;


    public RESTUtils(String URL) {
        Client client = ClientBuilder.newClient();
        webTarget = client.target(URL);
    }

    public String getTextPlain(String URL) {
        WebTarget resourceWebTarget = webTarget.path(URL);
        Invocation.Builder invocationBuilder = resourceWebTarget.request("text/plain");
        Response response = null;
        Log.logConfig("REST","Attempting request with URL:" + resourceWebTarget.getUri());
        try {
            response = invocationBuilder.get();
        } catch (ProcessingException e) {
            Log.logSevere("REST", "Cannot connect to REST service: " + e);
            System.exit(0);
        }
        checkForError(response,resourceWebTarget.getUri());
        String responseString = response.readEntity(String.class);
        Log.logConfig("REST","Text Returned from request '"+ URL + "' is: " + responseString);
        return responseString;
    }

    public void getCall(String URL) {
        WebTarget resourceWebTarget = webTarget.path(URL);
        Invocation.Builder invocationBuilder = resourceWebTarget.request("text/plain");
        Response response = null;
        Log.logConfig("REST","Attempting request with URL:" + resourceWebTarget.getUri());
        try {
            response = invocationBuilder.get();
        } catch (ProcessingException e) {
            Log.logSevere("REST", "Cannot connect to REST service: " + e);
            System.exit(0);
        }
        checkForError(response,resourceWebTarget.getUri());
    }

    public String getTextPlain(String URL,HashMap<String,String> queryParams) {
        WebTarget resourceWebTarget = webTarget.path(URL);
        for (HashMap.Entry<String, String> entry : queryParams.entrySet()) {
            resourceWebTarget = resourceWebTarget.queryParam(entry.getKey(), entry.getValue());
        }
        Invocation.Builder invocationBuilder = resourceWebTarget.request("text/plain");

        Log.logConfig("REST","Attempting request with URL:" + resourceWebTarget.getUri());
        Response response = null;
        try {
            response = invocationBuilder.get();
        } catch (ProcessingException e) {
            Log.logSevere("REST", "Cannot connect to REST service: " + e);
            System.exit(0);
        }
        checkForError(response,resourceWebTarget.getUri());
        String responseString = response.readEntity(String.class);
        Log.logConfig("REST","Text Returned from request '"+ URL + "' is: " + responseString);
        return responseString;
    }

    public String getJSON(String URL){
        WebTarget resourceWebTarget = webTarget.path(URL);
        Invocation.Builder invocationBuilder = resourceWebTarget.request("application/json");
        Response response = null;
        Log.logConfig("REST","Attempting request with URL:" + resourceWebTarget.getUri());
        try {
            response = invocationBuilder.get();
        } catch (ProcessingException e) {
            Log.logSevere("REST", "Cannot connect to REST service: " + e);
            System.exit(0);
        }
        checkForError(response,resourceWebTarget.getUri());
        String responseString = response.readEntity(String.class);
        Log.logConfig("REST","JSON Returned from request '"+ URL + "' is: " + responseString);
        return responseString;
    }

    public String getJSONPostJSON(String URL,String jsonString){
        WebTarget resourceWebTarget = webTarget.path(URL);
        Invocation.Builder invocationBuilder =  resourceWebTarget.request("application/json");
        Log.logConfig("REST","Sending POST REST request with json:" + jsonString);
        Response response = invocationBuilder.put(Entity.json(jsonString));
        checkForError(response,resourceWebTarget.getUri());
        String responseString = response.readEntity(String.class);
        Log.logConfig("REST","JSON Returned from request '"+ URL + "' is: " + responseString);
        return responseString;
    }

    public void getFile(String URL,String folder, String fileName,String fileExtention){
        java.nio.file.Path out = Paths.get(folder + "/" + fileName + "." + fileExtention);
        WebTarget resourceWebTarget = webTarget.path(URL);
        Invocation.Builder invocationBuilder = resourceWebTarget.request("application/octet-stream");
        Response response;
        Log.logConfig("REST","Attempting request with URL:" + resourceWebTarget.getUri());
        try {
            response = invocationBuilder.get();
            checkForError(response,resourceWebTarget.getUri());
            InputStream in = response.readEntity(InputStream.class);
            Files.copy(in, out, StandardCopyOption.REPLACE_EXISTING);
            in.close();
            Log.logConfig("REST","Downloaded file '" + fileName + "." + fileExtention + "'.");
        } catch (ProcessingException e) {
            Log.logSevere("REST", "Cannot connect to REST service: " + e);
            System.exit(0);
        } catch (IOException e) {
            Log.logSevere("REST", "Could not store file '" + fileName + "." + fileExtention + "' after download: " + e);
        }
    }

    private void checkForError(Response response,java.net.URI url) {
        if (response.getStatus() != Response.Status.OK.getStatusCode()) {
            Response.Status status = Response.Status.fromStatusCode(response.getStatus());
            WebApplicationException webAppException;
            switch (status) {
                case BAD_REQUEST:
                    webAppException = new BadRequestException();
                    break;
                case FORBIDDEN:
                    webAppException = new ForbiddenException();
                    break;
                case NOT_FOUND:
                    webAppException = new NotFoundException();
                    break;
                case NOT_ACCEPTABLE:
                    webAppException = new NotAcceptableException();
                    break;
                case UNSUPPORTED_MEDIA_TYPE:
                    webAppException = new NotSupportedException();
                    break;
                case INTERNAL_SERVER_ERROR:
                    webAppException = new InternalServerErrorException();
                    break;
                case SERVICE_UNAVAILABLE:
                    webAppException = new ServiceUnavailableException();
                    break;
                default:
                    webAppException = new WebApplicationException();
            }
            Log.logSevere("REST", "Error with request: " + webAppException.getMessage() + ". URL:" + url);
            System.exit(0);
        }
    }
}

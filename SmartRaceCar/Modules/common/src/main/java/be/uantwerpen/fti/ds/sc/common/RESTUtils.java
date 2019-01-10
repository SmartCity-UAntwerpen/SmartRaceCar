package be.uantwerpen.fti.ds.sc.common;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.ws.rs.*;
import javax.ws.rs.client.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;

import static javax.ws.rs.core.MediaType.*;

/**
 * Help model to deal with REST client requests. Uses Jersey library.
 */
public class RESTUtils
{
	private Logger log;
	private WebTarget webTarget; // URL to the domain of the REST service that is being connected to.


	/**
	 * Help model to deal with REST client requests. Uses Jersey library.
	 *
	 * @param URL to the REST service that is being connected to.
	 */
	public RESTUtils(String URL)
	{
		this.log = LoggerFactory.getLogger(this.getClass());

		Client client = ClientBuilder.newClient();
		this.webTarget = client.target(URL);
	}

	/**
	 * Perform GET request, expecting a textual answer (Can be anything non-binary such as HTML, JSON, plaintext, ...)
	 * @param endpoint
	 * @param expectedResponseType
	 * @return
	 * @throws ProcessingException
	 */
	public String get (String endpoint, MediaType expectedResponseType) throws ProcessingException, WebApplicationException
	{
		WebTarget resourceWebTarget = this.webTarget.path(endpoint);
		Invocation.Builder invocationBuilder = resourceWebTarget.request(expectedResponseType);
		Response response = null;
		this.log.debug("Attempting GET request: " + resourceWebTarget.getUri());

		try
		{
			response = invocationBuilder.get();
		}
		catch (ProcessingException pe)
		{
			this.log.error("Cannot process request to REST service (URI: " + resourceWebTarget.getUri() + ")", pe);
			throw pe;
		}

		this.checkForError(response, resourceWebTarget.getUri());
		String responseString = response.readEntity(String.class);
		this.log.debug("Text Returned from request '" + endpoint + "' is: " + responseString);
		return responseString;
	}

	/**
	 * Perform GET request without expecting an answer.
	 * @param endpoint
	 */
	public void get (String endpoint) throws WebApplicationException
	{
		WebTarget resourceWebTarget = this.webTarget.path(endpoint);
		Invocation.Builder invocationBuilder = resourceWebTarget.request(MediaType.TEXT_PLAIN);
		Response response = null;
		this.log.debug("Attempting GET request: " + resourceWebTarget.getUri());

		try
		{
			response = invocationBuilder.get();
		}
		catch (ProcessingException pe)
		{
			this.log.error("Cannot connect to REST service (URI: " + resourceWebTarget.getUri() + ")", pe);
			throw pe;
		}

		this.checkForError(response, resourceWebTarget.getUri());
	}

	/**
	 * Perform a POST request, request contains the given payload.
	 * The payload should be of type payloadType.
	 * Payload should be text of some form (HTML, JSON, plain text, ...)
	 * @param endpoint
	 * @param payload
	 * @param payloadType
	 * @return
	 */
	public String post(String endpoint, String payload, MediaType payloadType) throws IOException, WebApplicationException
	{
		WebTarget resourceWebTarget = this.webTarget.path(endpoint);
		Invocation.Builder invocationBuilder = resourceWebTarget.request("application/json");
		this.log.info("Attempting POST request: " + resourceWebTarget.getUri());
		Response response = null;

		try
		{
			if (payloadType == APPLICATION_JSON_TYPE)
			{
				response = invocationBuilder.put(Entity.json(payload));
			}
			else if ((payloadType == APPLICATION_XML_TYPE) || (payloadType == TEXT_XML_TYPE))
			{
				response = invocationBuilder.put(Entity.xml(payload));
			}
			else if (payloadType == APPLICATION_XHTML_XML_TYPE)
			{
				response = invocationBuilder.put(Entity.xhtml(payload));
			}
			else if (payloadType == TEXT_HTML_TYPE)
			{
				response = invocationBuilder.put(Entity.html(payload));
			}
			else if (payloadType == TEXT_PLAIN_TYPE)
			{
				response = invocationBuilder.put(Entity.text(payload));
			}
			else
			{
				String errorString = "Got unsupported payload type: " + payloadType;
				this.log.error(errorString);
				throw new ProcessingException (errorString);
			}
		}
		catch (ProcessingException e)
		{
			String errorString = "Cannot connect to REST service (URL: \"" + resourceWebTarget.getUri() + "\": ";
			this.log.error(errorString, e);
			throw new IOException(errorString);
		}

		this.checkForError(response, resourceWebTarget.getUri());
		String responseString = response.readEntity(String.class);
		this.log.info("JSON Returned from request '" + endpoint + "' is: " + responseString);
		return responseString;
	}

	/**
	 * Send a POST request without expecting an answer.
	 * @param endpoint
	 * @return
	 * @throws IOException
	 */
	public String post(String endpoint) throws WebApplicationException
	{
		this.log.debug("Attempting POST request with URL: \"" + endpoint + "\"");

		WebTarget resourceWebTarget = this.webTarget.path(endpoint);

		Invocation.Builder invocationBuilder = resourceWebTarget.request();

		Response response = invocationBuilder.post(Entity.text(""));

		checkForError(response, resourceWebTarget.getUri());

		String responseString = response.readEntity(String.class);

		this.log.debug("POST Request got response: \"" + responseString + "\"");

		return responseString;
	}

	/**
	 * Send a DELETE request without expecting an answer.
	 * @param endpoint
	 * @return
	 * @throws IOException
	 */
	public String delete(String endpoint) throws WebApplicationException
	{
		this.log.debug("Attempting DELETE request with URL: \"" + endpoint + "\"");

		WebTarget resourceWebTarget = this.webTarget.path(endpoint);

		Invocation.Builder invocationBuilder = resourceWebTarget.request();

		Response response = invocationBuilder.delete();

		checkForError(response, resourceWebTarget.getUri());

		String responseString = response.readEntity(String.class);

		this.log.debug("DELETE Request got response \"" + responseString + "\"");

		return responseString;
	}

	/**
	 * REST GET request of the receive a response of the Octet-stream to download a file.
	 *
	 * @param URL           Path of the GET request.
	 * @param folder        The path to download the file towards.
	 * @param fileName      Filename the downloaded file should get.
	 * @param fileExtention File extention the downloaded file should get.
	 */
	public void getFile(String URL, String folder, String fileName, String fileExtention) throws ProcessingException, IOException
	{
		java.nio.file.Path out = Paths.get(folder + "/" + fileName + "." + fileExtention);
		WebTarget resourceWebTarget = this.webTarget.path(URL);
		Invocation.Builder invocationBuilder = resourceWebTarget.request("application/octet-stream");
		Response response;
		this.log.info("Attempting GET(file) request with URL: " + resourceWebTarget.getUri());
		try
		{
			response = invocationBuilder.get();
			checkForError(response, resourceWebTarget.getUri());
			InputStream in = response.readEntity(InputStream.class);
			Files.copy(in, out, StandardCopyOption.REPLACE_EXISTING);
			in.close();
			this.log.info("Downloaded file '" + fileName + "." + fileExtention + "'.");
		}
		catch (ProcessingException pe)
		{
			this.log.error("Cannot connect to REST service.", pe);
			throw pe;
		}
		catch (IOException ioe)
		{
			this.log.error("Could not store file '" + fileName + "." + fileExtention + "' after download.", ioe);
			throw ioe;
		}
	}

	/**
	 * Help function to analyze the REST response to get the HTTP status and check which code it was. If it was an
	 * error code this method will throw the right exception.
	 *
	 * @param response The REST response.
	 * @param url      The URL that was being reached.
	 */
	private void checkForError(Response response, java.net.URI url) throws WebApplicationException
	{
		if (response.getStatus() != Response.Status.OK.getStatusCode())
		{
			Response.Status status = Response.Status.fromStatusCode(response.getStatus());
			WebApplicationException webAppException;
			switch (status)
			{
				case BAD_REQUEST:
					webAppException = new BadRequestException(response.readEntity(String.class));
					break;
				case FORBIDDEN:
					webAppException = new ForbiddenException(response.readEntity(String.class));
					break;
				case NOT_FOUND:
					webAppException = new NotFoundException(response.readEntity(String.class));
					break;
				case NOT_ACCEPTABLE:
					webAppException = new NotAcceptableException(response.readEntity(String.class));
					break;
				case UNSUPPORTED_MEDIA_TYPE:
					webAppException = new NotSupportedException(response.readEntity(String.class));
					break;
				case INTERNAL_SERVER_ERROR:
					webAppException = new InternalServerErrorException(response.readEntity(String.class));
					break;
				case SERVICE_UNAVAILABLE:
					webAppException = new ServiceUnavailableException(response.readEntity(String.class));
					break;
				default:
					webAppException = new WebApplicationException(response.readEntity(String.class));
					break;
			}

			this.log.error("Error with request for URL:" + url, webAppException);
			throw webAppException;
		}
	}
}

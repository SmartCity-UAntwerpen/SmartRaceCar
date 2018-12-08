package be.uantwerpen.fti.ds.sc.smartracecar.common;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Help class to look for specific files.
 */
public class FileUtils
{
	private LogbackWrapper log;

	private String fileNameToSearch; // Name of the file that is being searched.
	private List<String> result = new ArrayList<>(); // List of all found files matching the searched files. Full path is stored.

	public FileUtils()
	{
		this.log = new LogbackWrapper(FileUtils.class);
	}

	/**
	 * /* Get the name of the file being searched.
	 *
	 * @return String with the name of the file being searched.
	 */
	public String getFileNameToSearch()
	{
		return fileNameToSearch;
	}

	/**
	 * /* Get the name of the file being searched.
	 *
	 * @param fileNameToSearch The String with the name of the file being searched.
	 */
	public void setFileNameToSearch(String fileNameToSearch)
	{
		this.fileNameToSearch = fileNameToSearch;
	}

	/**
	 * /* Get the list will all found files.
	 *
	 * @return List with all found files matching the searched filename. Full path.
	 */
	public List<String> getResult()
	{
		return result;
	}

	/**
	 * /* Execute a search in given directory to search.
	 * It will loop through all subdirectories.
	 *
	 * @param directory        Path of the directory to be searched.
	 * @param fileNameToSearch Name of the file to be searched.
	 */
	public void searchDirectory(File directory, String fileNameToSearch)
	{

		setFileNameToSearch(fileNameToSearch);

		if (directory.isDirectory())
		{
			search(directory);
		} else
		{
			this.log.error("FILESEARCH", "Not a directory. Cannot search.");
		}
	}

	/**
	 * /* Search in given directory for the searched file.
	 *
	 * @param file Path of the directory to be searched.
	 */
	private void search(File file)
	{

		if (file.isDirectory())
		{

			//do you have permission to read this directory?
			if (file.canRead())
			{
				for (File temp : file.listFiles())
				{
					if (temp.isDirectory())
					{
						search(temp);
					} else
					{
						if (getFileNameToSearch().equals(temp.getName().toLowerCase()))
						{
							result.add(temp.getPath().toString().replace("\\maps.xml", ""));
						}

					}
				}

			} else
			{
				this.log.error("FILESEARCH", "No permission to search for files.");
			}
		}

	}

}

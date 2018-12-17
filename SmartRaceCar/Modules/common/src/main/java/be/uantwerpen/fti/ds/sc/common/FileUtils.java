package be.uantwerpen.fti.ds.sc.common;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Help class to look for specific files.
 */
public class FileUtils
{
	private Logger log;

	private String fileNameToSearch; // Name of the file that is being searched.
	private List<String> result; // List of all found files matching the searched files. Full path is stored.

	public FileUtils()
	{
		this.log = LoggerFactory.getLogger(FileUtils.class);
		this.result = new ArrayList<>();
	}

	/**
	 * /* Get the name of the file being searched.
	 *
	 * @return String with the name of the file being searched.
	 */
	public String getFileNameToSearch()
	{
		return this.fileNameToSearch;
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
		return this.result;
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
		try
		{
			this.log.info("Searching for directory with file " + fileNameToSearch + " in " + directory.getCanonicalPath());
		} catch (IOException e)
		{
			e.printStackTrace();
		}
		setFileNameToSearch(fileNameToSearch);

		if (directory.isDirectory())
		{
			this.search(directory);
		} else
		{
			this.log.error(directory.getName() + " is not a directory. Cannot search.");
		}
	}

	/**
	 * /* Search in given directory for the searched file.
	 *
	 * @param file Path of the directory to be searched.
	 */
	private void search(File file)
	{
		this.log.info("Searching for " + file.getPath());
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
					}
					else
					{
						if (getFileNameToSearch().equals(temp.getName().toLowerCase()))
						{
							this.result.add(temp.getParent());
							this.log.info("added " + temp.getParent() + " to results");
						}

					}
				}

			} else
			{
				this.log.error("No permission to search for files.");
			}
		}

	}

}

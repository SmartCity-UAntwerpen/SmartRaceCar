package be.uantwerpen.fti.ds.sc.smartracecar.common;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class FileUtils {

    private String fileNameToSearch;
    private List<String> result = new ArrayList<String>();

    public String getFileNameToSearch() {
        return fileNameToSearch;
    }

    public void setFileNameToSearch(String fileNameToSearch) {
        this.fileNameToSearch = fileNameToSearch;
    }

    public List<String> getResult() {
        return result;
    }

    public void searchDirectory(File directory, String fileNameToSearch) {

        setFileNameToSearch(fileNameToSearch);

        if (directory.isDirectory()) {
            search(directory);
        } else {
            Log.logSevere("FILESEARCH","Not a directory. Cannot search.");
        }

    }

    private void search(File file) {

        if (file.isDirectory()) {

            //do you have permission to read this directory?
            if (file.canRead()) {
                for (File temp : file.listFiles()) {
                    if (temp.isDirectory()) {
                        search(temp);
                    } else {
                        if (getFileNameToSearch().equals(temp.getName().toLowerCase())) {
                            result.add(temp.getPath().toString().replace("\\maps.xml", ""));
                        }

                    }
                }

            } else {
                Log.logSevere("FILESEARCH","No permission to search for files.");
            }
        }

    }

}

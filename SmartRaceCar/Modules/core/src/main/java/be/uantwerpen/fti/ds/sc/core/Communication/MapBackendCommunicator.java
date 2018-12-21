package be.uantwerpen.fti.ds.sc.core.Communication;

public interface MapBackendCommunicator
{
	public String getMapName();
	public void downloadMap(String mapName);
}

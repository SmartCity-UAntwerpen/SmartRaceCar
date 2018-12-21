package be.uantwerpen.fti.ds.sc.core.Communication;

public interface BackendMapCommunicator
{
	public String getMapName();
	public void downloadMap(String mapName);
}

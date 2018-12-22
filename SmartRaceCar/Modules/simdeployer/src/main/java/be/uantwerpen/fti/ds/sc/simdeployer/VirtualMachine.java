package be.uantwerpen.fti.ds.sc.simdeployer;

import java.io.IOException;
import java.util.List;

public interface VirtualMachine
{
	public void run(List<String> args) throws IOException;

	public void stop();
}

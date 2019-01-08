package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

import java.net.InetAddress;
import java.net.UnknownHostException;

@SpringBootApplication
@EnableScheduling
public class RacecarBackend
{
	public static void main(String[] args)
	{
		SpringApplication.run(RacecarBackend.class, args);

		Logger log = LoggerFactory.getLogger(RacecarBackend.class);

		try
		{
			InetAddress ip = InetAddress.getLocalHost();
			log.info("Backend IP Address: " + ip.getHostAddress());
		}
		catch (UnknownHostException uhe)
		{
			log.warn("Failed to determine Backend IP Address: ", uhe);
		}
	}
}

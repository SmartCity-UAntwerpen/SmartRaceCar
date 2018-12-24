package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class RacecarBackend
{
	public static void main(String[] args)
	{
		SpringApplication.run(RacecarBackend.class, args);
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend.maps;

import be.uantwerpen.fti.ds.sc.common.WayPoint;

import javax.persistence.*;

@Entity
@Table(name = "points")
public class Waypoint
{
	@Id
	@GeneratedValue (strategy = GenerationType.IDENTITY)
	private long id;

	private float x;
	private float y;
	private float z;
	private float w;

	@Column(name = "mapname")
	private String mapName;

	public Waypoint()
	{
	}

	public Waypoint (float x, float y, float z, float w, String mapName)
	{
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
		this.mapName = mapName;
	}

	public long getId()
	{
		return this.id;
	}

	public void setId(long id)
	{
		this.id = id;
	}

	public float getX()
	{
		return this.x;
	}

	public void setX(float x)
	{
		this.x = x;
	}

	public float getY()
	{
		return this.y;
	}

	public void setY(float y)
	{
		this.y = y;
	}

	public float getZ()
	{
		return this.z;
	}

	public void setZ(float z)
	{
		this.z = z;
	}

	public float getW()
	{
		return this.w;
	}

	public void setW(float w)
	{
		this.w = w;
	}

	public String getMapName()
	{
		return this.mapName;
	}

	public void setMapName(String mapName)
	{
		this.mapName = mapName;
	}
}

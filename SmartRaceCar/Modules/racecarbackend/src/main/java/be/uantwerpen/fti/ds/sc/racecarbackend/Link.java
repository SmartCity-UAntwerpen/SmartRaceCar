package be.uantwerpen.fti.ds.sc.racecarbackend;

import java.util.Objects;

public class Link
{
	private long id1;
	private long id2;

	public Link (long id1, long id2)
	{
		this.id1 = id1;
		this.id2 = id2;
	}

	/**
	 * We need to override comparison because the order of the ID's doesn't matter, as long as the values are the same, we're talking about the same link.
	 * @param other
	 * @return
	 */
	@Override
	public boolean equals (Object other)
	{
		if (!(other instanceof Link))
		{
			return false;
		}

		Link otherLink = (Link) other;

		// Case where same IDs in the same fields
		if ((this.id1 == otherLink.id1) && (this.id2 == otherLink.id2))
		{
			return true;
		}

		// Case where same IDs in opposite fields
		if ((this.id1 == otherLink.id2) && (this.id2 == otherLink.id1))
		{
			return true;
		}

		return false;
	}

	@Override
	public int hashCode()
	{
		long smallestId = this.id1 < this.id2 ? this.id1 : this.id2;
		long largestId  = this.id1 < this.id2 ? this.id2 : this.id1;

		return Objects.hash(smallestId, largestId);
	}

	@Override
	public String toString()
	{
		return this.id1 + " -> " + this.id2;
	}
}

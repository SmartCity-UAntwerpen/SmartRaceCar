package be.uantwerpen.fti.ds.sc.common.commands;

import java.util.HashSet;
import java.util.Set;

public class Range
{
	private long low;
	private long high;

	public Range(long number)
	{
		this.low = number;
		this.high = number;
	}

	public Range(long low, long high)
	{
		this.low = low;
		this.high = high;
	}

	public Set<Long> generate()
	{
		Set<Long> set = new HashSet<>();

		for (long i = this.low; i <= this.high; ++i)
		{
			set.add(i);
		}

		return set;
	}

	@Override
	public String toString()
	{
		if (1 <= (this.high - this.low))
		{
			return this.low + "..." + this.high;
		}
		else
		{
			return Long.toString(this.low);
		}
	}

	public static Range parseRange(String rangeString)
	{
		if (rangeString.contains("..."))
		{
			String[] array = rangeString.split("\\.{3}");

			if (array.length > 2)
			{
				throw new NumberFormatException("Failed to parse range from String \"" + rangeString + "\", got " + array.length + " parts after splitting on \"...\", expected 2.");
			}

			return new Range(Long.parseLong(array[0]), Long.parseLong(array[1]));
		}
		else
		{
			return new Range(Long.parseLong(rangeString));
		}
	}
}

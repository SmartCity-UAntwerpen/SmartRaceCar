package be.uantwerpen.fti.ds.sc.racecarbackend;

public class CheckedIndexOutOfBoundsException extends Exception
{
	public CheckedIndexOutOfBoundsException()
	{
		super();
	}

	public CheckedIndexOutOfBoundsException(String s)
	{
		super(s);
	}

	public CheckedIndexOutOfBoundsException(String s, Throwable throwable)
	{
		super(s, throwable);
	}

	@Override
	public String getMessage()
	{
		return super.getMessage();
	}

	@Override
	public synchronized Throwable getCause()
	{
		return super.getCause();
	}

	@Override
	public void printStackTrace()
	{
		super.printStackTrace();
	}
}

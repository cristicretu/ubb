package IsPrime;

import java.util.Arrays;
import java.util.List;

import IsPrime.MyValueException;

public class LongSeq {

	private List<Integer> l;
	private List<Integer> lSeq;
	private int start, length;
	
	
	 public LongSeq()
	 
	{
		System.out.println("Longest sequence vida ...");	
	}
	public void setSequence(List<Integer> l)
	{
		this.l=l;
		
	}
	public LongSeq(List<Integer> newL)
	{
		this.lSeq = Arrays.asList();		
		this.l = newL;
		this.start=-1;
		this.length=0;
		System.out.println("Longest sequence ...");		
	}
	public int getStart()
	{
		return this.start;
	}
	public int getLength()
	{
		return this.length;
		
	}
	public boolean isPrimeStub(int n) throws MyValueException{
		boolean isPrime=true;
		////l.add(5);l.add(7);l.add(11);l.add(2);l.add(13);l.add(23);
		if ((n==0) || (n==1) || (n==4) || (n==12) || (n==15))
			return isPrime=false;
		else
		if ((n==2) || (n==5) || (n==7) || (n==11) ||(n==13) ||(n==23))
			return isPrime=true;
		return isPrime;
	}

	public boolean isPrime(int n) throws MyValueException{
		boolean b = true;
		if(n<0)
		{
			throw new MyValueException("data not valid");
		}
		if(n<2)
		{
			b=false;			
		}
		else
		{
			int i=2;
			while (b && (i<= (n/2)))
			{
				if ((n % i) == 0)
				{
					b=false;
				}
				i++;
			}			
		}
		//System.out.println("Numarul"+ String.valueOf(n)+ "este prim:"+ String.valueOf(b));
		return b;
	}	
	
	public void SolveLongestSequence() throws MyValueException
	{
		//System.out.println("in sir ... initial ... nr elemente=  "+this.l.size());
		int posI=-1, lengthI=0, i=0;
		int posF=-1, lengthF=0;
		//System.out.println("l.size="+this.l.size());
		while(i<this.l.size())
		{
			//if(isPrime(this.l.get(i).intValue())==true)
			if(isPrimeStub(this.l.get(i).intValue())==true)
			{
				if(posI==-1)
				{
					posI=i;
					lengthI=1;					
				}
				else
					lengthI++;
			//	System.out.println("inca prim...valori intermediare: pozI=" + Integer.toString(posI)+ " si lengthI="+Integer.toString(lengthI));
			}
			else
			{
				if(lengthI>lengthF)
				{
					lengthF=lengthI;
					posF = posI;
					//System.out.println("valori intermediare: pozI=" + Integer.toString(posI)+ " si lengthI="+Integer.toString(lengthI));
				}
				posI=-1; lengthI=0;	
			}
   		 //  System.out.println("valori intermediare: pozI=" + Integer.toString(posI)+ " si lengthI="+Integer.toString(lengthI));
			i++;
			
		}
		if(lengthI>lengthF)
		{
			lengthF=lengthI;
			posF = posI;
		}
		//System.out.println("in metoda, la final, valorile sunt: pozF=" + Integer.toString(posF)+ " si lengthF="+Integer.toString(lengthF));
		
		this.start =posF;
		this.length=lengthF;
		//System.out.println("in sir ... la final ... nr elemente=  "+this.l.size());
		}
		
		
	
}

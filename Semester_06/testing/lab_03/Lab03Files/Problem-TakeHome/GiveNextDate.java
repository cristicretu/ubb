
public class GiveNextDate{
	
	private int month,day,year;
	
	
	public GiveNextDate(int m, int d, int y){
		//Initialize the mont, day, and year
		month = m;
		day = d;
		year = y;
	}
		
	//This method returns a string depicting the next date in the from MM/DD/YY 	
	public String run(){
		
		//Restrictions that the year must have the following invariant: 1801 <= year <= 2021
		
		if (day < 1 || month < 1 || month >12 || year < 1801 || year > 2021)
			return "invalid Input Date";
		
		//these variables will hold the proper values for the nextDate's day, month, and year values, respectively
		int tomorrowDay = day;
		int tomorrowMonth = month;
		int tomorrowYear = year;
		
		//Is this month with 31 days?
		if(isThirtyOneDayMonth(month))
		{
			if(day < 31) //if the day is not 31, just increment the day
				tomorrowDay = day +1;
			else{ //day = 31, set tomorrow's day to 1 and increment the month
				tomorrowDay = 1;
				tomorrowMonth = month + 1;
			}
		}
		//is this month a month with 30 days?
		else if (isThirtyDayMonth(month))
		{
			if(day <30) //if the day is not 30, just increment the day
				tomorrowDay = day + 1;
			else {
				if(day == 30){
					tomorrowDay = 1;
					tomorrowMonth = month +1;
				}
				else //invalid input ... too many days
					return "Invalid Input Date";
			}
		}
		//is this month December?
		else if(isDecember(month))
		{
			if (day <= 31) //if the day is not 31, just increment the next day
				tomorrowDay = day +1;
			else { //day is 31, reset the day and month to 1 and increment the year
				tomorrowDay = 1;
				tomorrowMonth = 1;
				if(year == 2021) //make sure the next year is within ht permissible range
					return "Invalid Next Year";
				else  //it was a permissible year - go ahead and increment the year
					tomorrowYear = year +1;
			}
				
		}
		//is this month February? we need to check for leap years and such
		else if(isFebruary(month))
		{
			if(day < 28) //just a standard day - increment the day
				tomorrowDay =day +1;
			else {
				if(day == 28) {  //if this is not a leap year, reset day and increment the day 
					if(isLeapYear(year)) //was a leap year
						tomorrowDay = 29;
					else {  //was not a leap year
						tomorrowDay = 1;
						tomorrowMonth = 3;
					}
				}
				else if(day == 29){ //29th date of February
					if(isLeapYear(year)){  //AND a leap year - reset the day to 1, month to 3
						tomorrowDay = 1;
						tomorrowMonth = 3;
					}
					else
						return "Invalid Input Date";
				}
				else if(day > 29) //invalid input as February will never have more than 29 days
					return "Invalid Input Date";
			}
		}
		//return the string representing the nextDate, in the form MM/DD/YY
		return tomorrowMonth + "/" + tomorrowDay + "/" + tomorrowYear;

	}
	
	
	//This method returns true is 'month' corresponds to a month that contains 31 days, excluding December	 	
	private static boolean isThirtyOneDayMonth(int month)
	{
		return month == 1 || month == 3 || month == 5 || month == 8 || month == 10;
	}
	
	
	//This method returns true if 'month' corresponds to a month that contains 30 days.		 	
	private static boolean isThirtyDayMonth(int month)
	{
		return month == 4 || month == 6 || month == 9 || month == 11;
	}
	
	
	//This method returns true if 'month' corresponds to December	
	private static boolean isDecember(int month)
	{
		return month == 12;
	}
	
	
	//This method returns true if 'month' corresponds to  February	
	private static boolean isFebruary(int month)
	{
		return month == 2;
	}
	
	
	//This method returns true if 'year' corresponds to a leap year. 
	/* It works like this:
	 *   *If the year is not a century year and divisible by 4,	then it is a leap year
	 *	 *If the year is a century year, it is a leap year if and only if it is divisible by 400
	 */
	
	private static boolean isLeapYear(int year)
	{
		if((year % 100) == 0)
			return (year % 400) == 0;
		else
			return (year % 4) == 0;
	}
	
	
	
	public static void main(String[] args)
	{
		GiveNextDate q;
		int m, d, y;
        
        
		m = 5;
		d = 31;
		y = 1900;
		
		q =	new GiveNextDate(m,d,y);			
		
		System.out.println("Date before: " + m +"/"+ d +"/"+ y);		
		System.out.println("Date after: " + q.run());
		
	}


}



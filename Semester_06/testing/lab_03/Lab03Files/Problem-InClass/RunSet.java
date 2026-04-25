
public class RunSet {

	
	public static void main(String[] args) {
		
			
			Set s = new Set(5);
			s.AddAValue(3);
			s.AddAValue(2);
			s.AddAValue(1);		
			for (int i=0;i<s.nVS;i++)
				System.out.println(s.vS[i]+", ");
	
	}

}

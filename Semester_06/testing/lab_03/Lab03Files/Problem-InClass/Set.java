
public class Set {

	int vS[];
	int nVS;
	
	Set(int capacity) {
		vS = new int[capacity];
		nVS = 0;
	}
	
	boolean AddAValue(int newVal) {
		if (IsInTheSet(newVal))
			return false;
		vS[nVS++]=newVal;
		return true;
	}
	
	
	
	public boolean IsInTheSet(int checkVal) {
		boolean b=false;
		int i=0;
		while((i<nVS) && (vS[i]!=checkVal)) {
			if (checkVal == vS[i]) {
				b=true;
			}
			i++;
		}	
		return b;
	}
	
	
}

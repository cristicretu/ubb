package org.example.pages;

import net.thucydides.core.pages.PageObject;
import org.openqa.selenium.By;

public class WikipediaResultPage extends PageObject {

    public String getPageBodyText() {
        return find(By.tagName("body")).getText();
    }

    public String getCurrentPageUrl() {
        return getDriver().getCurrentUrl();
    }
}

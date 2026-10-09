package org.example.pages;

import net.serenitybdd.core.annotations.findby.FindBy;
import net.serenitybdd.core.pages.WebElementFacade;
import net.thucydides.core.annotations.DefaultUrl;
import net.thucydides.core.pages.PageObject;

@DefaultUrl("https://en.wikipedia.org/wiki/Main_Page")
public class WikipediaHomePage extends PageObject {

    @FindBy(name = "search")
    private WebElementFacade searchInput;

    public void enter_search_term(String keyword) {
        searchInput.waitUntilVisible();
        searchInput.clear();
        searchInput.typeAndEnter(keyword);
    }
}

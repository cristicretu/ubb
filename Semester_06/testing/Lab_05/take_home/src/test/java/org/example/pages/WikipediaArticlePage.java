package org.example.pages;

import net.thucydides.core.pages.PageObject;

public class WikipediaArticlePage extends PageObject {

    private static final String ARTICLE_URL_TEMPLATE = "https://en.wikipedia.org/wiki/%s";

    public void open_article(String articleSlug) {
        getDriver().get(String.format(ARTICLE_URL_TEMPLATE, articleSlug));
    }
}

using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Mvc.Filters;

namespace CarDealershipApi.Attributes
{
    public class AuthRequiredAttribute : AuthorizeAttribute, IAuthorizationFilter
    {
        public void OnAuthorization(AuthorizationFilterContext context)
        {
            // Check if user is authenticated
            var user = context.HttpContext.User;
            if (!user.Identity.IsAuthenticated)
            {
                // Return 401 Unauthorized if not logged in
                context.Result = new UnauthorizedResult();
                return;
            }
        }
    }
} 